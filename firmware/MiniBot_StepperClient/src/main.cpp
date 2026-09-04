/*
A FreeRTOS powered controller for a tiny 2 wheel robot.
*/

#include <Arduino.h>
#undef LOG_LOCAL_LEVEL
#define LOG_LOCAL_LEVEL LOG_LEVEL_MAIN
#include "battery_monitor.h"
#include "device_id.h"
#include "esp_log.h"
#include "esp_now_communicator.h"
#include "kinematics_controller.h"
#include "led_status.h"
#include "motion_queue.h"
#include "motor_test_queue.h"
#include "position_estimator.h"
#include "robot.h"

static const char *TAG = "MAIN";

static Robot robot;
static MotionQueue motion_queue = NULL;
static MotorTestQueue motor_test_queue = NULL;

static TaskHandle_t kinematics_task_handle = NULL;
static TaskHandle_t communicator_task_handle = NULL;
static TaskHandle_t position_estimator_sensor_task_handle = NULL;
static TaskHandle_t position_estimator_calc_task_handle = NULL;
static TaskHandle_t battery_monitor_task_handle = NULL;
static TaskHandle_t led_status_task_handle = NULL;

static bool initialize_status_led(void) {
  BaseType_t task_created =
      xTaskCreate(LedStatus_Task, "LedStatus", 1024, (void *)&robot,
                  LED_STATUS_PRIORITY, &led_status_task_handle);
  if (task_created != pdPASS) {
    ESP_LOGE(TAG, "Failed to create LED Status task");
    return false;
  }
  ESP_LOGI(TAG, "LED Status task created successfully");

  if (!LedStatus_Init(LED_PIN)) {
    ESP_LOGE(TAG, "Failed to initialize LED status");
    return false;
  }
  ESP_LOGI(TAG, "LED status initialized successfully");

  return true;
}

static bool create_tasks(void) {
  BaseType_t task_created;

  task_created = xTaskCreate(KinematicsController_Task, "KinematicsController",
                             8192, (void *)&robot, KINEMATICS_TASK_PRIORITY,
                             &kinematics_task_handle);
  if (task_created != pdPASS) {
    ESP_LOGE(TAG, "Failed to create Kinematics Controller task");
    return false;
  }
  ESP_LOGI(TAG, "Kinematics Controller task created successfully");

  task_created = xTaskCreate(EspNowCommunicator_Task, "EspNowCommunicator",
                             2048, (void *)&robot, ESP_NOW_COMM_PRIORITY,
                             &communicator_task_handle);
  if (task_created != pdPASS) {
    ESP_LOGE(TAG, "Failed to create ESP-NOW Communicator task");
    return false;
  }
  ESP_LOGI(TAG, "ESP-NOW Communicator task created successfully");

  task_created =
      xTaskCreate(PositionEstimator_SensorTask, "PositionEstimatorSensor", 4096,
                  (void *)&robot, POSITION_EST_SENSOR_PRIORITY,
                  &position_estimator_sensor_task_handle);
  if (task_created != pdPASS) {
    ESP_LOGE(TAG, "Failed to create Position Estimator Sensor task");
    return false;
  }
  ESP_LOGI(TAG, "Position Estimator Sensor task created successfully");

  task_created = xTaskCreate(
      PositionEstimator_CalcTask, "PositionEstimatorCalc", 4096, (void *)&robot,
      POSITION_EST_CALC_PRIORITY, &position_estimator_calc_task_handle);
  if (task_created != pdPASS) {
    ESP_LOGE(TAG, "Failed to create Position Estimator Calc task");
    return false;
  }
  ESP_LOGI(TAG, "Position Estimator Calc task created successfully");

  static BatteryMonitorParams battery_monitor_params = {
      .robot = &robot,
      .kinematics_task = kinematics_task_handle,
      .communicator_task = communicator_task_handle,
      .position_estimator_sensor_task = position_estimator_sensor_task_handle,
      .position_estimator_calc_task = position_estimator_calc_task_handle,
  };

  task_created =
      xTaskCreate(BatteryMonitor_Task, "BatteryMonitor", 2048,
                  (void *)&battery_monitor_params, BATTERY_MONITOR_PRIORITY,
                  &battery_monitor_task_handle);
  if (task_created != pdPASS) {
    ESP_LOGE(TAG, "Failed to create Battery Monitor task");
    return false;
  }
  ESP_LOGI(TAG, "Battery Monitor task created successfully");

  return true;
}

static bool initialize_modules(void) {
  motion_queue = MotionQueue_Create(MOTION_QUEUE_SIZE);
  if (motion_queue == NULL) {
    ESP_LOGE(TAG, "Failed to create motion queue");
    return false;
  }
  ESP_LOGI(TAG, "Motion queue created successfully");

  motor_test_queue = MotorTestQueue_Create(MOTION_QUEUE_SIZE);
  if (motor_test_queue == NULL) {
    ESP_LOGE(TAG, "Failed to create motor test queue");
    return false;
  }
  ESP_LOGI(TAG, "Motor test queue created successfully");

  if (!robot.initialize()) {
    ESP_LOGE(TAG, "Failed to initialize robot");
    return false;
  }
  ESP_LOGI(TAG, "Robot initialized successfully");

  if (!KinematicsController_Init(motion_queue, motor_test_queue)) {
    ESP_LOGE(TAG, "Failed to initialize kinematics controller");
    return false;
  }
  ESP_LOGI(TAG, "Kinematics controller initialized successfully");

  if (!EspNowCommunicator_Init(motion_queue, motor_test_queue)) {
    ESP_LOGE(TAG, "Failed to initialize ESP-NOW communicator");
    return false;
  }
  ESP_LOGI(TAG, "ESP-NOW communicator initialized successfully");

  if (!PositionEstimator_Init()) {
    ESP_LOGE(TAG, "Failed to initialize position estimator");
    return false;
  }
  ESP_LOGI(TAG, "Position estimator initialized successfully");

  if (!BatteryMonitor_Init(BATTERY_VOLTAGE_PIN)) {
    ESP_LOGE(TAG, "Failed to initialize battery monitor");
    return false;
  }
  ESP_LOGI(TAG, "Battery monitor initialized successfully");

  return true;
}

void setup() {
  Serial.begin(115200);
  delay(100);

  // Initialize status led first to indicate if any errors occur
  if (!initialize_status_led()) {
    while (1) {
      ESP_LOGE(TAG, "Failed to initialize status LED");
      delay(1000);
    }
  }
  LedStatus_SetStatus(LED_STATUS_STARTUP);

  // Set component log levels
  esp_log_level_set("*", ESP_LOG_ERROR);
  esp_log_level_set("MAIN", (esp_log_level_t)LOG_LEVEL_MAIN);
  esp_log_level_set("BATTERY", (esp_log_level_t)LOG_LEVEL_BATTERY);
  esp_log_level_set("DEVICE_ID", (esp_log_level_t)LOG_LEVEL_DEVICE_ID);
  esp_log_level_set("ESPNOW", (esp_log_level_t)LOG_LEVEL_ESPNOW);
  esp_log_level_set("KINEMATICS", (esp_log_level_t)LOG_LEVEL_KINEMATICS);
  esp_log_level_set("POS_EST", (esp_log_level_t)LOG_LEVEL_POS_EST);
  esp_log_level_set("MMC5633", (esp_log_level_t)LOG_LEVEL_MMC5633);
  esp_log_level_set("ROBOT", (esp_log_level_t)LOG_LEVEL_ROBOT);
  ESP_LOGI(TAG, "\n\n=== MiniBot Stepper Client ===");

  ESP_LOGI(TAG, "Initializing FreeRTOS framework...");
  if (!initialize_modules()) {
    LedStatus_SetStatus(LED_STATUS_ERROR);
    while (1) {
      ESP_LOGE(TAG, "FATAL: Module initialization failed");
      delay(1000);
    }
  }

  ESP_LOGI(TAG, "Modules initialized successfully");
  ESP_LOGI(TAG, "Creating FreeRTOS tasks...");

  if (!create_tasks()) {
    LedStatus_SetStatus(LED_STATUS_ERROR);
    while (1) {
      ESP_LOGE(TAG, "FATAL: Task creation failed");
      delay(1000);
    }
  }

  ESP_LOGI(TAG, "All tasks created successfully");
  ESP_LOGI(TAG,
           "Core 0: ESP-NOW, Position Estimator, Battery Monitor, LED Status");
  ESP_LOGI(TAG, "Core 1: Kinematics Controller");

  setCpuFrequencyMhz(80);

  // Write device ID to NVS, ony do this one time. Persistant across reflashes.
  // TODO create a function to set ID based on current position on chess board
  // setDeviceID(0x0d);

  uint8_t device_id = getDeviceID();
  if (device_id == 0xFF) {
    ESP_LOGW(TAG, "Device ID not configured!");
    ESP_LOGW(TAG, "Set ID by calling: setDeviceID(id) where id is 0x01-0xFE");
    LedStatus_SetStatus(LED_STATUS_BREATHING_FAST);
  } else {
    ESP_LOGI(TAG, "Device ID: 0x%02X", device_id);
    LedStatus_SetStatus(LED_STATUS_READY);
  }
}

void loop() { vTaskDelay(pdMS_TO_TICKS(1000)); }
