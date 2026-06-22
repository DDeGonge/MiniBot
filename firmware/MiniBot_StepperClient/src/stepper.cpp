#undef LOG_LOCAL_LEVEL
#define LOG_LOCAL_LEVEL LOG_LEVEL_ROBOT
#include "stepper.h"
#include "config.h"
#include "esp_log.h"

static const char *TAG = "ROBOT";

bool StepperDriver::initialize(uint8_t step, uint8_t dir, uint8_t enable,
                               uint8_t reset, bool reverse) {
  step_pin = step;
  dir_pin = dir;
  enable_pin = enable;
  reset_pin = reset;
  reverse_motor = reverse;
  enabled = false;
  current_direction = 1;
  current_step_count = 0;

  // Configure pins as outputs
  pinMode(step_pin, OUTPUT);
  pinMode(dir_pin, OUTPUT);
  pinMode(enable_pin, OUTPUT);
  pinMode(reset_pin, OUTPUT);

  digitalWrite(enable_pin, LOW);
  digitalWrite(reset_pin, LOW);
  digitalWrite(step_pin, LOW);
  digitalWrite(dir_pin, LOW);

  initTimer();

  return true;
}

bool StepperDriver::setMicrostepping(bool step_lvl, bool dir_lvl) {
  // See config.h for microstepping settings
  digitalWrite(step_pin, step_lvl);
  digitalWrite(dir_pin, dir_lvl);
  return true;
}

bool StepperDriver::resetDriver() {
  disable();
  digitalWrite(reset_pin, LOW);
  vTaskDelay(pdMS_TO_TICKS(5));
  digitalWrite(reset_pin, HIGH);
  vTaskDelay(pdMS_TO_TICKS(5));
  return true;
}

bool StepperDriver::enable() {
  digitalWrite(enable_pin, HIGH);
  enabled = true;
  return true;
}

bool StepperDriver::disable() {
  digitalWrite(enable_pin, LOW);
  enabled = false;
  return true;
}

bool StepperDriver::setDirection(bool direction) {
  current_direction = direction ? 1 : -1;
  bool actual_direction = reverse_motor ? !direction : direction;
  digitalWrite(dir_pin, actual_direction);
  return true;
}

bool StepperDriver::step() {
  if (!enabled) {
    return false;
  }

  digitalWrite(step_pin, HIGH);
  delayMicroseconds(1); // STSPIN220 requires >100ns pulse width
  digitalWrite(step_pin, LOW);

  current_step_count += current_direction;

  return true;
}

void StepperDriver::stepISR(void *arg) {
  StepperDriver *instance = (StepperDriver *)arg;
  instance->step();
  instance->timer_steps_taken++;
  if (instance->timer_steps_taken >= instance->timer_steps_target &&
      instance->timer_steps_target > 0) {
    instance->stopTimer();
  }
}

bool StepperDriver::initTimer() {
  esp_timer_create_args_t timer_args = {
      .callback = stepISR,
      .arg = this,
      .dispatch_method = ESP_TIMER_TASK,
      .name = "stepper_timer",
  };

  if (esp_timer_create(&timer_args, &this->s_step_timer) != ESP_OK) {
    ESP_LOGE(TAG, "Failed to create stepper timer");
    return false;
  }
  timer_initialized = true;
  return true;
}

bool StepperDriver::configureTimer(float velocity_rad_s, float steps_per_rad,
                                   int steps_to_take) {
  if (!timer_initialized) {
    ESP_LOGE(TAG, "Timer not initialized. Set timer first.");
    return false;
  }

  // Calculate step interval in microseconds
  step_interval_us = (uint32_t)fmax(
      1, (uint32_t)round(1000000.0f / (velocity_rad_s * steps_per_rad)) - 1);
  step_interval_us =
      fmin(step_interval_us, 100000); // Cap at 100ms to avoid shenanigans

  // Stop any existing transmission
  if (timer_running) {
    stopTimer();
  }

  timer_steps_target = steps_to_take;
  timer_steps_taken = 0;

  return true;
};

bool StepperDriver::startTimer() {
  if (!timer_initialized || timer_running)
    return false;
  esp_timer_start_periodic(this->s_step_timer, step_interval_us);
  return true;
}

bool StepperDriver::stopTimer() {
  if (!timer_initialized)
    return false;
  esp_timer_stop(this->s_step_timer);
  return true;
}
