#include "battery_monitor.h"
#include "esp_now_communicator.h"
#undef LOG_LOCAL_LEVEL
#define LOG_LOCAL_LEVEL LOG_LEVEL_BATTERY
#include "config.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "led_status.h"

static const char *TAG = "BATTERY";

static uint8_t battery_adc_pin = 0;
static RollingAverage<BATTERY_AVG_WINDOW_SIZE> battery_avg_v;
static bool low_battery_alerted = false;
static esp_timer_handle_t charge_timeout_timer = NULL;

static void charge_timeout_callback(void *arg) {
  ESP_LOGI(TAG, "Charge timeout reached after %d s; LED set to steady on",
           CHARGE_TIMER_S);
  LedStatus_SetStatus(LED_STATUS_CHARGED);
}

bool BatteryMonitor_Init(uint8_t adc_pin) {
  battery_adc_pin = adc_pin;
  pinMode(battery_adc_pin, INPUT);

  if (charge_timeout_timer == NULL) {
    esp_timer_create_args_t timer_args = {
        .callback = charge_timeout_callback,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "charge_timeout",
    };
    if (esp_timer_create(&timer_args, &charge_timeout_timer) != ESP_OK) {
      ESP_LOGE(TAG, "Failed to create charge timeout timer");
      return false;
    }
  }

  return true;
}

void BatteryMonitor_RecordVoltage() {
  uint16_t adc_raw = analogRead(battery_adc_pin);
  float batt_voltage =
      (adc_raw / 4095.0f) * 3.3f * BATTERY_VOLTAGE_DIVIDER_RATIO;
  battery_avg_v.add(batt_voltage);
}

void BatteryMonitor_Task(void *pvParameters) {
  BatteryMonitorParams *params = (BatteryMonitorParams *)pvParameters;

  if (params == NULL || params->robot == NULL) {
    vTaskDelete(NULL);
    return;
  }

  Robot *robot = params->robot;
  bool tasks_suspended = false;

  vTaskDelay(pdMS_TO_TICKS(100));

  while (1) {
    BatteryMonitor_RecordVoltage();
    float current_voltage = battery_avg_v.avg();
    robot->setBatteryVoltage(current_voltage);

    // Check for low battery condition and send alert
    if (robot->getBatteryCritical()) {
      if (!low_battery_alerted) {
        LedStatus_SetStatus(LED_STATUS_LOW_BATTERY);
        ESP_LOGW(TAG, "LOW BATTERY DETECTED: %.2fV", current_voltage);
        if (EspNowCommunicator_SendAlert(ERR_LOW_BATTERY)) {
          ESP_LOGI(TAG, "Low battery alert sent successfully");
          low_battery_alerted = true;
        } else {
          ESP_LOGE(TAG, "Failed to send low battery alert");
        }
      }
    } else {
      low_battery_alerted = false;
    }

    // Determine whether other tasks should be suspended
    bool is_charging = (!ENABLE_BOT_WHILE_CHARGING && robot->getBatteryCharging());
    bool should_suspend = robot->getBatteryCritical() || is_charging;

    if (should_suspend && !tasks_suspended) {
      ESP_LOGW(TAG, "Battery condition (%.2fV) requires suspending tasks.",
               current_voltage);
      robot->disableMotors();
      if (params->kinematics_task != NULL)
        vTaskSuspend(params->kinematics_task);
      if (params->communicator_task != NULL)
        vTaskSuspend(params->communicator_task);
      if (params->position_estimator_sensor_task != NULL)
        vTaskSuspend(params->position_estimator_sensor_task);
      if (params->position_estimator_calc_task != NULL)
        vTaskSuspend(params->position_estimator_calc_task);
      tasks_suspended = true;
      if (is_charging) {
        LedStatus_SetStatus(LED_STATUS_BREATHING);
        if (esp_timer_is_active(charge_timeout_timer)) {
          esp_timer_stop(charge_timeout_timer);
        }
        esp_timer_start_once(charge_timeout_timer,
                             (uint64_t)CHARGE_TIMER_S * 1000000ULL);
      }
    } else if (!should_suspend && tasks_suspended) {
      ESP_LOGI(TAG, "Battery voltage (%.2fV) recovered. Resuming tasks.",
               current_voltage);
      if (esp_timer_is_active(charge_timeout_timer)) {
        esp_timer_stop(charge_timeout_timer);
      }
      if (params->kinematics_task != NULL)
        vTaskResume(params->kinematics_task);
      if (params->communicator_task != NULL)
        vTaskResume(params->communicator_task);
      if (params->position_estimator_sensor_task != NULL)
        vTaskResume(params->position_estimator_sensor_task);
      if (params->position_estimator_calc_task != NULL)
        vTaskResume(params->position_estimator_calc_task);
      tasks_suspended = false;
    }

    vTaskDelay(pdMS_TO_TICKS(BATTERY_POLL_INTERVAL_MS));
  }
}
