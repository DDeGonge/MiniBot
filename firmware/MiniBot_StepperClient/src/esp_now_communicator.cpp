#include "esp_now_communicator.h"
#include "device_id.h"
#include "position_estimator.h"
#undef LOG_LOCAL_LEVEL
#define LOG_LOCAL_LEVEL LOG_LEVEL_ESPNOW
#include "config.h"
#include "esp_log.h"
#include <cmath>

static const char *TAG = "ESPNOW";

static MotionQueue comm_queue = NULL;
static MotorTestQueue motor_test_queue = NULL;
static EspNowReceiveCallback receive_callback = nullptr;
static Robot *g_robot_ptr = NULL;
static uint8_t last_sender_mac[6] = {0};
static bool station_mac_established = false;
static bool has_pending_completion_ack = false;

static volatile bool waiting_for_pos_sync = false;
static volatile int64_t pos_sync_deadline_us = 0;
static volatile bool pos_sync_received = false;
static volatile int64_t pos_sync_best_receive_time_us = 0;
static volatile uint32_t pos_sync_best_ttnf_us = 0;
static volatile int64_t pos_sync_candidate_time_us =
    0; // written in recv_cb, read in handler

static int64_t last_sync_received_us =
    0; // cleared at boot; set on each successful PosSync
static bool duty_cycle_active = false;
static uint16_t last_accepted_seq_num = 0;
static bool seq_num_initialized = false;

static void esp_now_recv_cb(const uint8_t *mac_addr, const uint8_t *data,
                            int len);
static void esp_now_message_handler(const uint8_t *mac_addr,
                                    const uint8_t *data, int len);
static bool ensure_peer_exists(const uint8_t *mac_addr);
static bool send_ack_message(const uint8_t *mac_addr);
static void send_completion_ack_if_pending();
static void pos_sync_store(int64_t receive_time_us, uint32_t ttnf_us);
static void set_duty_cycle(bool enable);

bool EspNowCommunicator_Init(MotionQueue motion_queue,
                             MotorTestQueue motor_test_q) {
  if (motion_queue == NULL || motor_test_q == NULL) {
    return false;
  }

  comm_queue = motion_queue;
  motor_test_queue = motor_test_q;

  WiFi.mode(WIFI_STA);
  vTaskDelay(pdMS_TO_TICKS(100));
  WiFi.setTxPower(WIFI_POWER);
  ESP_LOGD(TAG, "WiFi MAC: %s", WiFi.macAddress().c_str());

  if (esp_now_init() != ESP_OK) {
    ESP_LOGE(TAG, "ESP-NOW initialization failed");
    return false;
  }

  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_now_register_recv_cb(esp_now_recv_cb);

  esp_now_peer_info_t peer = {};
  memset(&peer, 0, sizeof(esp_now_peer_info_t));
  memset(peer.peer_addr, 0xFF, ESP_NOW_ETH_ALEN);
  peer.channel = WIFI_CHANNEL;

  if (esp_now_add_peer(&peer) != ESP_OK) {
    ESP_LOGW(TAG, "Failed to add broadcast peer");
  }

  EspNowCommunicator_RegisterCallback(esp_now_message_handler);
  ESP_LOGI(TAG, "ESP-NOW ready");
  return true;
}

static void pos_sync_store(int64_t receive_time_us, uint32_t ttnf_us) {
  pos_sync_best_receive_time_us = receive_time_us;
  pos_sync_best_ttnf_us = ttnf_us;
  pos_sync_received = true;
}

static void set_duty_cycle(bool enable) {
  if (enable == duty_cycle_active)
    return;
  if (enable) {
    esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
    esp_now_set_wake_window(ESPNOW_WAKE_WINDOW_MS *
                            1000); // API takes microseconds
    esp_wifi_set_connectionless_wake_interval(ESPNOW_WAKE_INTERVAL_MS);
    ESP_LOGI(TAG, "Duty cycle enabled (%d ms window / %d ms interval)",
             ESPNOW_WAKE_WINDOW_MS, ESPNOW_WAKE_INTERVAL_MS);
  } else {
    esp_wifi_set_ps(WIFI_PS_NONE);
    ESP_LOGI(TAG, "Duty cycle disabled — radio always on");
  }
  duty_cycle_active = enable;
}

static void esp_now_recv_cb(const uint8_t *mac_addr, const uint8_t *data,
                            int len) {
  if (mac_addr == NULL || data == NULL) {
    return;
  }

  // Capture timestamp as early as possible for PosSync timing accuracy
  if (len >= 2 && waiting_for_pos_sync && data[1] == MSG_TYPE_POS_SYNC) {
    pos_sync_candidate_time_us = esp_timer_get_time();
  }

  if (receive_callback) {
    receive_callback(mac_addr, data, len);
  }
}

static bool ensure_peer_exists(const uint8_t *mac_addr) {
  if (mac_addr == NULL) {
    return false;
  }

  if (!esp_now_is_peer_exist(mac_addr)) {
    esp_now_peer_info_t peer_info = {};
    memcpy(peer_info.peer_addr, mac_addr, ESP_NOW_ETH_ALEN);
    peer_info.channel = WIFI_CHANNEL;
    peer_info.encrypt = false;

    esp_err_t add_result = esp_now_add_peer(&peer_info);
    if (add_result != ESP_OK) {
      ESP_LOGE(TAG, "Failed to add peer (error: %d)", add_result);
      return false;
    }
  }
  return true;
}

static bool record_station_mac(const uint8_t *mac_addr) {
  if (mac_addr == NULL) {
    return false;
  }

  memcpy(last_sender_mac, mac_addr, 6);
  station_mac_established = true;
  return true;
}

static bool send_ack_message(const uint8_t *mac_addr) {
  if (mac_addr == NULL || g_robot_ptr == NULL) {
    return false;
  }

  float x, y, theta;
  g_robot_ptr->getPosition(&x, &y, &theta);

  AckMessage ack = {};
  ack.responderID = getDeviceID();
  ack.msg_type = MSG_TYPE_ACK_MESSAGE;
  ack.timestamp = millis();
  ack.x = x;
  ack.y = y;
  ack.orientation_rad = theta;
  ack.battery_voltage = g_robot_ptr->getBatteryVoltage();

  if (!ensure_peer_exists(mac_addr)) {
    return false;
  }

  esp_err_t result =
      esp_now_send(mac_addr, (uint8_t *)&ack, sizeof(AckMessage));
  if (result != ESP_OK) {
    ESP_LOGE(TAG, "Failed to send ack (err: %d)", result);
    return false;
  }
  return true;
}

static bool send_nack_message(const uint8_t *mac_addr,
                              const uint8_t error_type) {
  if (mac_addr == NULL || g_robot_ptr == NULL) {
    return false;
  }

  float x, y, theta;
  g_robot_ptr->getPosition(&x, &y, &theta);

  NackMessage nack = {};
  nack.responderID = getDeviceID();
  nack.msg_type = MSG_TYPE_NACK_MESSAGE;
  nack.timestamp = millis();
  nack.err_type = error_type;

  if (!ensure_peer_exists(mac_addr)) {
    return false;
  }

  esp_err_t result =
      esp_now_send(mac_addr, (uint8_t *)&nack, sizeof(NackMessage));
  if (result != ESP_OK) {
    ESP_LOGE(TAG, "Failed to send nack (err: %d)", result);
    return false;
  }
  return true;
}

static bool send_mag_field_response(const uint8_t *mac_addr) {
  if (mac_addr == NULL) {
    return false;
  }

  float x, y, z;
  if (!PositionEstimator_GetLatestMagneticField(&x, &y, &z)) {
    ESP_LOGW(TAG, "Could not retrieve magnetometer readings");
    return send_nack_message(mac_addr, ERR_ROBOT_UNAVAILABLE);
  }

  MagneticFieldResponse response = {};
  response.responderID = getDeviceID();
  response.msg_type = MSG_TYPE_MAG_REQUEST;
  response.timestamp = millis();
  response.field_x_gauss = x;
  response.field_y_gauss = y;
  response.field_z_gauss = z;

  if (!ensure_peer_exists(mac_addr)) {
    return false;
  }

  esp_err_t result = esp_now_send(mac_addr, (uint8_t *)&response,
                                  sizeof(MagneticFieldResponse));
  if (result != ESP_OK) {
    ESP_LOGE(TAG, "Failed to send mag field response (err: %d)", result);
    return false;
  }
  return true;
}

static void send_completion_ack_if_pending() {
  if (has_pending_completion_ack) {
    ESP_LOGD(TAG, "Sending motion completion acknowledgment");
    send_ack_message(last_sender_mac);
    has_pending_completion_ack = false;
  }
}

static void esp_now_message_handler(const uint8_t *mac_addr,
                                    const uint8_t *data, int len) {
  if (len < 2 || comm_queue == NULL) {
    return;
  }

  uint8_t target_id = data[0];
  uint8_t msg_type = data[1];

  if (target_id != getDeviceID() && target_id != 0xFF) {
    return;
  }

  if (!station_mac_established) {
    if (!record_station_mac(mac_addr)) {
      ESP_LOGE(TAG, "Failed to record station MAC address");
      return;
    }
    ESP_LOGD(TAG, "Recorded station MAC: %02X:%02X:%02X:%02X:%02X:%02X",
             last_sender_mac[0], last_sender_mac[1], last_sender_mac[2],
             last_sender_mac[3], last_sender_mac[4], last_sender_mac[5]);
  }

  switch (msg_type) {
  case MSG_TYPE_POSITION_COMMAND: {
    if (len < sizeof(PositionCommand)) {
      ESP_LOGE(TAG, "PositionCommand too short");
      send_nack_message(mac_addr, ERR_INVALID_MSG_SIZE);
      return;
    }

    PositionCommand *cmd = (PositionCommand *)data;
    MotionCommand motion_cmd = {cmd->target_x_mm, cmd->target_y_mm,
                                cmd->target_a_rad, cmd->move_duration_ms};

    if (MotionQueue_Enqueue(comm_queue, &motion_cmd)) {
      has_pending_completion_ack = true;
      send_ack_message(mac_addr);
    } else {
      ESP_LOGE(TAG, "Motion queue full");
      send_nack_message(mac_addr, ERR_QUEUE_FULL);
    }
    break;
  }

  case MSG_TYPE_MOT_TEST_COMMAND: {
    if (len < sizeof(MotTestCommand)) {
      ESP_LOGE(TAG, "MotTestCommand too short");
      send_nack_message(mac_addr, ERR_INVALID_MSG_SIZE);
      return;
    }

    MotTestCommand *test_cmd = (MotTestCommand *)data;

    if (test_cmd->enabled) {
      // Convert int8_t command to rad/s (scale from -128..127 using max stepper
      // velocity)
      float m0_vel_rad_s =
          (((float)test_cmd->m0_vel / 128.0f) * STEPPER_MAX_VELOCITY_MM_S) /
          WHEEL_RADIUS_MM;
      float m1_vel_rad_s =
          (((float)test_cmd->m1_vel / 128.0f) * STEPPER_MAX_VELOCITY_MM_S) /
          WHEEL_RADIUS_MM;

      MotorTestRequest motor_req = {m0_vel_rad_s, m1_vel_rad_s};

      if (MotorTestQueue_Enqueue(motor_test_queue, &motor_req)) {
        ESP_LOGD(TAG, "Motor test command queued: M0=%.2f rad/s, M1=%.2f rad/s",
                 m0_vel_rad_s, m1_vel_rad_s);
        // send_ack_message(mac_addr);
      } else {
        ESP_LOGE(TAG, "Motor test queue full");
        // send_nack_message(mac_addr, ERR_QUEUE_FULL);
      }
    } else {
      // Stop motor test - send immediate stop signal via zero velocity command
      MotorTestRequest motor_req = {0.0f, 0.0f};
      MotorTestQueue_Enqueue(motor_test_queue, &motor_req);
      ESP_LOGD(TAG, "Motor test stop command queued");
      // send_ack_message(mac_addr);
    }
    break;
  }

  case MSG_TYPE_POSITION_REQUEST: {
    if (len < sizeof(PositionRequest)) {
      ESP_LOGE(TAG, "PositionRequest too short");
      send_nack_message(mac_addr, ERR_INVALID_MSG_SIZE);
      return;
    }

    if (g_robot_ptr == NULL) {
      ESP_LOGE(TAG, "Robot pointer unavailable");
      send_nack_message(mac_addr, ERR_ROBOT_UNAVAILABLE);
      return;
    }

    send_ack_message(mac_addr);
    break;
  }

  case MSG_TYPE_ACK_MESSAGE: {
    if (len < sizeof(AckMessage)) {
      ESP_LOGE(TAG, "AckMessage too short");
      send_nack_message(mac_addr, ERR_INVALID_MSG_SIZE);
      return;
    }
    ESP_LOGD(TAG, "Received acknowledgment message");
    send_nack_message(mac_addr, ERR_NOT_IMPLEMENTED);
    break;
  }

  case MSG_TYPE_MAG_REQUEST: {
    if (len < sizeof(MagneticFieldRequest)) {
      ESP_LOGE(TAG, "MagneticFieldRequest too short");
      send_nack_message(mac_addr, ERR_INVALID_MSG_SIZE);
      return;
    }
    ESP_LOGD(TAG, "Received magnetometer field request");
    send_mag_field_response(mac_addr);
    break;
  }

  case MSG_TYPE_POS_SYNC_COMMAND: {
    if (len < sizeof(PosSyncCommand)) {
      ESP_LOGE(TAG, "PosSyncCommand too short");
      send_nack_message(mac_addr, ERR_INVALID_MSG_SIZE);
      return;
    }
    if (waiting_for_pos_sync) {
      ESP_LOGW(TAG, "PosSync wait already in progress");
      send_nack_message(mac_addr, ERR_ROBOT_UNAVAILABLE);
      return;
    }
    PosSyncCommand *cmd = (PosSyncCommand *)data;
    pos_sync_received = false;
    pos_sync_deadline_us =
        esp_timer_get_time() + (int64_t)cmd->timeout_ms * 1000;
    waiting_for_pos_sync = true;
    // EspNowCommunicator_Task will busy-wait for MSG_TYPE_POS_SYNC and send
    // ack/nack
    break;
  }

  case MSG_TYPE_POS_SYNC: {
    if (waiting_for_pos_sync && len >= (int)sizeof(PosSync)) {
      PosSync *msg = (PosSync *)data;
      uint32_t ttnf = msg->next_frame_us;
      int64_t candidate = pos_sync_candidate_time_us + (int64_t)ttnf;
      // Keep the earliest-arriving pulse — minimum latency = best accuracy
      if (!pos_sync_received || candidate < pos_sync_best_receive_time_us) {
        pos_sync_best_receive_time_us = candidate;
        pos_sync_best_ttnf_us = ttnf; // not used anywhere just for tracking
        pos_sync_received = true;
        // Serial.printf("PosSYNC (new best): rt=%lld us\tttnf=%u us\n", (long
        // long)candidate, ttnf);
      }
    }
    break;
  }

  default:
    send_nack_message(mac_addr, ERR_UNKNOWN_MSG);
    break;
  }
}

bool EspNowCommunicator_RegisterCallback(EspNowReceiveCallback callback) {
  receive_callback = callback;
  return true;
}

bool EspNowCommunicator_SendAlert(uint8_t error_type) {
  // Only send alert if we have a valid last sender
  // Check if the MAC address is not all zeros
  bool has_valid_sender = false;
  for (int i = 0; i < 6; i++) {
    if (last_sender_mac[i] != 0) {
      has_valid_sender = true;
      break;
    }
  }

  if (!has_valid_sender) {
    ESP_LOGW(TAG, "No valid sender MAC for alert");
    return false;
  }

  return send_nack_message(last_sender_mac, error_type);
}

void EspNowCommunicator_Task(void *pvParameters) {
  Robot *robot = (Robot *)pvParameters;

  if (comm_queue == NULL || robot == NULL) {
    vTaskDelete(NULL);
    return;
  }

  g_robot_ptr = robot;

  while (1) {
    if (has_pending_completion_ack && !g_robot_ptr->isMoving()) {
      send_completion_ack_if_pending();
    }

    // Sleep through the deadline — recv_cb runs in the WiFi driver context
    if (waiting_for_pos_sync) {
      int64_t remaining_us = pos_sync_deadline_us - esp_timer_get_time();
      if (remaining_us > 0) {
        vTaskDelay(pdMS_TO_TICKS((remaining_us + 999) / 1000 + 1));
      }

      if (pos_sync_received) {
        PositionEstimator_SetSyncTime(pos_sync_best_receive_time_us);
        last_sync_received_us = esp_timer_get_time();

        // Random delay to spread acks from multiple robots. Idk if I need
        // this..
        vTaskDelay(pdMS_TO_TICKS(esp_random() % 101));
        send_ack_message(last_sender_mac);
      } else {
        ESP_LOGW(TAG, "PosSync timeout -- no MSG_TYPE_POS_SYNC received");
        send_nack_message(last_sender_mac, ERR_SYNC_TIMEOUT);
      }

      pos_sync_received = false;
      waiting_for_pos_sync = false;
    }

    // Enable duty cycling only when sync is fresh; fall back to always-on if
    // sync is stale so the server can still reach the bot to re-establish sync
    bool sync_is_fresh = (last_sync_received_us != 0) &&
                         ((esp_timer_get_time() - last_sync_received_us) <
                          (int64_t)SYNC_DUTY_CYCLE_TIMEOUT_MS * 1000LL);
    set_duty_cycle(sync_is_fresh);

#if SPAM_POSITION
    if (station_mac_established) {
      send_ack_message(last_sender_mac);
    }
#endif

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
