#include "ota_updater.h"
#include "esp_now_communicator.h"
#include "led_status.h"
#include "robot.h"
#undef LOG_LOCAL_LEVEL
#define LOG_LOCAL_LEVEL LOG_LEVEL_ESPNOW
#include "config.h"
#include "esp_log.h"
#include <HTTPClient.h>
#include <Update.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <nvs_flash.h>
#include <string.h>

static const char *TAG = "OTA";

static SemaphoreHandle_t ota_command_sem = NULL;
static OtaCommand pending_cmd;
static uint8_t pending_mac[6];
static bool nvs_initialized = false;

static uint32_t crc32_table[256];
static bool crc32_table_ready = false;

// ============================================================
// CRC32 (zlib/IEEE 802.3), matching the Server + Coordinator implementations
// ============================================================
static void init_crc32_table() {
  for (uint32_t i = 0; i < 256; i++) {
    uint32_t c = i;
    for (int k = 0; k < 8; k++) {
      c = (c & 1) ? (0xEDB88320U ^ (c >> 1)) : (c >> 1);
    }
    crc32_table[i] = c;
  }
  crc32_table_ready = true;
}

static uint32_t crc32_update(uint32_t crc, const uint8_t *buf, size_t len) {
  if (!crc32_table_ready) {
    init_crc32_table();
  }
  for (size_t i = 0; i < len; i++) {
    crc = crc32_table[(crc ^ buf[i]) & 0xFF] ^ (crc >> 8);
  }
  return crc;
}

// ============================================================
// Persisted firmware version (raw NVS, mirrors device_id.cpp's pattern)
// ============================================================
static void ensure_nvs_initialized() {
  if (nvs_initialized) {
    return;
  }
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    nvs_flash_erase();
    err = nvs_flash_init();
  }
  nvs_initialized = true;
}

static uint32_t get_current_version() {
  ensure_nvs_initialized();
  nvs_handle_t handle;
  if (nvs_open("ota", NVS_READONLY, &handle) != ESP_OK) {
    return 0; // namespace doesn't exist yet -> never updated before
  }
  uint32_t version = 0;
  nvs_get_u32(handle, "fw_version", &version);
  nvs_close(handle);
  return version;
}

static void set_current_version(uint32_t version) {
  ensure_nvs_initialized();
  nvs_handle_t handle;
  if (nvs_open("ota", NVS_READWRITE, &handle) != ESP_OK) {
    ESP_LOGE(TAG, "Failed to open NVS to persist firmware version");
    return;
  }
  if (nvs_set_u32(handle, "fw_version", version) != ESP_OK) {
    ESP_LOGE(TAG, "Failed to persist firmware version");
  }
  nvs_commit(handle);
  nvs_close(handle);
}

// ============================================================
// Public API
// ============================================================

bool OtaUpdater_Init() {
  ota_command_sem = xSemaphoreCreateBinary();
  if (ota_command_sem == NULL) {
    return false;
  }
  ensure_nvs_initialized();
  return true;
}

void OtaUpdater_OnCommandReceived(const uint8_t *mac_addr,
                                 const OtaCommand *cmd) {
  if (mac_addr == NULL || cmd == NULL || ota_command_sem == NULL) {
    return;
  }
  memcpy(&pending_cmd, cmd, sizeof(OtaCommand));
  memcpy(pending_mac, mac_addr, 6);
  xSemaphoreGive(ota_command_sem);
}

// ============================================================
// Download + flash sequence. Assumes WiFi is already in STA mode.
// ============================================================
static uint8_t perform_update(const OtaCommand &cmd) {
  ESP_LOGI(TAG, "Joining %s...", OTA_WIFI_SSID);
  WiFi.begin(OTA_WIFI_SSID, OTA_WIFI_PASSWORD);
  uint32_t wifi_start_ms = millis();
  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - wifi_start_ms > OTA_WIFI_CONNECT_TIMEOUT_MS) {
      ESP_LOGE(TAG, "WiFi connect to %s timed out", OTA_WIFI_SSID);
      return OTA_RESULT_WIFI_FAILED;
    }
    vTaskDelay(pdMS_TO_TICKS(200));
  }
  ESP_LOGI(TAG, "Connected, IP=%s", WiFi.localIP().toString().c_str());

  HTTPClient http;
  http.setTimeout(OTA_HTTP_TIMEOUT_MS);
  if (!http.begin(OTA_SERVER_URL)) {
    ESP_LOGE(TAG, "HTTP begin failed");
    return OTA_RESULT_DOWNLOAD_FAILED;
  }

  int http_code = http.GET();
  if (http_code != HTTP_CODE_OK) {
    ESP_LOGE(TAG, "HTTP GET failed: %d", http_code);
    http.end();
    return OTA_RESULT_DOWNLOAD_FAILED;
  }

  int content_len = http.getSize();
  if (content_len <= 0 || (uint32_t)content_len != cmd.fw_size) {
    ESP_LOGE(TAG, "Unexpected content length: %d (expected %u)", content_len,
             cmd.fw_size);
    http.end();
    return OTA_RESULT_DOWNLOAD_FAILED;
  }

  if (!Update.begin(cmd.fw_size)) {
    ESP_LOGE(TAG, "Update.begin failed: %s", Update.errorString());
    http.end();
    return OTA_RESULT_FLASH_FAILED;
  }

  WiFiClient *stream = http.getStreamPtr();
  uint8_t buf[OTA_DOWNLOAD_CHUNK_SIZE];
  uint32_t received = 0;
  uint32_t running_crc = 0xFFFFFFFFU;
  uint32_t last_data_ms = millis();
  uint8_t result = OTA_RESULT_SUCCESS;

  while (received < cmd.fw_size) {
    size_t avail = stream->available();
    if (avail == 0) {
      if (!http.connected() ||
          (millis() - last_data_ms) > (uint32_t)OTA_HTTP_TIMEOUT_MS) {
        ESP_LOGE(TAG, "HTTP stream stalled at %u/%u bytes", received,
                 cmd.fw_size);
        result = OTA_RESULT_DOWNLOAD_FAILED;
        break;
      }
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    size_t to_read = avail < sizeof(buf) ? avail : sizeof(buf);
    uint32_t remaining = cmd.fw_size - received;
    if (to_read > remaining) {
      to_read = remaining;
    }

    int n = stream->readBytes(buf, to_read);
    if (n <= 0) {
      continue;
    }
    if (Update.write(buf, n) != (size_t)n) {
      ESP_LOGE(TAG, "Flash write failed at %u bytes", received);
      result = OTA_RESULT_FLASH_FAILED;
      break;
    }
    running_crc = crc32_update(running_crc, buf, n);
    received += n;
    last_data_ms = millis();
  }
  http.end();

  if (result != OTA_RESULT_SUCCESS) {
    Update.abort();
    return result;
  }

  uint32_t final_crc = running_crc ^ 0xFFFFFFFFU;
  if (final_crc != cmd.fw_crc32) {
    ESP_LOGE(TAG, "CRC mismatch: got 0x%08X want 0x%08X", final_crc,
             cmd.fw_crc32);
    Update.abort();
    return OTA_RESULT_CRC_MISMATCH;
  }

  if (!Update.end(true)) {
    ESP_LOGE(TAG, "Update.end failed: %s", Update.errorString());
    return OTA_RESULT_FLASH_FAILED;
  }

  ESP_LOGW(TAG, "OTA flash complete (%u bytes, CRC verified)", received);
  return OTA_RESULT_SUCCESS;
}

void OtaUpdater_Task(void *pvParameters) {
  OtaUpdaterParams *params = (OtaUpdaterParams *)pvParameters;
  if (params == NULL || params->robot == NULL) {
    vTaskDelete(NULL);
    return;
  }

  for (;;) {
    if (xSemaphoreTake(ota_command_sem, portMAX_DELAY) != pdTRUE) {
      continue;
    }

    OtaCommand cmd;
    uint8_t mac[6];
    memcpy(&cmd, &pending_cmd, sizeof(cmd));
    memcpy(mac, pending_mac, 6);

    uint32_t current_version = get_current_version();
    if (cmd.fw_version == current_version) {
      ESP_LOGI(TAG, "Already running version %u; skipping OTA",
               current_version);
      EspNowCommunicator_SendOtaResult(mac, OTA_RESULT_ALREADY_CURRENT,
                                       current_version);
      continue;
    }

    ESP_LOGW(TAG, "Starting OTA: current=%u new=%u size=%u", current_version,
             cmd.fw_version, cmd.fw_size);

    LedStatus prev_led_status = LedStatus_GetStatus();
    LedStatus_SetStatus(LED_STATUS_OTA_UPDATING);

    params->robot->disableMotors();
    if (params->kinematics_task != NULL)
      vTaskSuspend(params->kinematics_task);
    if (params->communicator_task != NULL)
      vTaskSuspend(params->communicator_task);
    if (params->position_estimator_sensor_task != NULL)
      vTaskSuspend(params->position_estimator_sensor_task);
    if (params->position_estimator_calc_task != NULL)
      vTaskSuspend(params->position_estimator_calc_task);

    uint8_t result = perform_update(cmd);

    // Reply over ESP-NOW while the OTA AP session is still up. Sending
    // after WiFi.disconnect() drops the datagram on both success and
    // failure paths, so the server would never see the outcome.
    if (result == OTA_RESULT_SUCCESS) {
      set_current_version(cmd.fw_version);
      EspNowCommunicator_SendOtaResult(mac, result, cmd.fw_version);
      ESP_LOGW(TAG, "OTA succeeded, rebooting...");
    } else {
      ESP_LOGE(TAG, "OTA failed with code %d", result);
      EspNowCommunicator_SendOtaResult(mac, result, current_version);
    }

    WiFi.disconnect();
    esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

    if (result == OTA_RESULT_SUCCESS) {
      vTaskDelay(pdMS_TO_TICKS(250));
      esp_restart();
    }

    // WiFi.begin + WiFi.disconnect during the OTA reset the ESP-IDF wake
    // window / interval that set_duty_cycle() had configured in the
    // communicator module. Force a re-application so ESP-NOW broadcasts
    // keep working for the rest of the boot.
    EspNowCommunicator_RestoreDutyCycle();

    params->robot->enableMotors();
    if (params->kinematics_task != NULL)
      vTaskResume(params->kinematics_task);
    if (params->communicator_task != NULL)
      vTaskResume(params->communicator_task);
    if (params->position_estimator_sensor_task != NULL)
      vTaskResume(params->position_estimator_sensor_task);
    if (params->position_estimator_calc_task != NULL)
      vTaskResume(params->position_estimator_calc_task);
    LedStatus_SetStatus(prev_led_status);
  }
}
