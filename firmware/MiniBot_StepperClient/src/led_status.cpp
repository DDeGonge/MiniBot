#include "led_status.h"

static uint8_t led_pin = 0;
static LedStatus current_led_status = LED_STATUS_STARTUP;
static uint8_t led_brightness = 127;
static bool led_initialized = false;

#define LED_MIN_BRIGHTNESS 4

bool LedStatus_Init(uint8_t pin) {
  led_pin = pin;
  pinMode(led_pin, OUTPUT);
  analogWrite(led_pin, 0);
  led_initialized = true;
  return true;
}

void LedStatus_SetStatus(LedStatus status) { current_led_status = status; }

LedStatus LedStatus_GetStatus(void) { return current_led_status; }

static void blink_pattern(uint8_t count, uint16_t on_ms, uint16_t off_ms) {
  for (uint8_t i = 0; i < count; i++) {
    analogWrite(led_pin, led_brightness);
    vTaskDelay(pdMS_TO_TICKS(on_ms));
    analogWrite(led_pin, 0);
    vTaskDelay(pdMS_TO_TICKS(off_ms));
  }
}

static void breathe_step(uint8_t pin, uint8_t step, uint8_t total_steps) {
  // Triangle wave: ramp up then back down, clamped between LED_MIN_BRIGHTNESS
  // and led_brightness
  uint8_t half = total_steps / 2;
  uint8_t val;
  if (step < half) {
    val = LED_MIN_BRIGHTNESS +
          (uint8_t)((uint16_t)step * (led_brightness - LED_MIN_BRIGHTNESS) /
                    half);
  } else {
    val = LED_MIN_BRIGHTNESS +
          (uint8_t)((uint16_t)(total_steps - step) *
                    (led_brightness - LED_MIN_BRIGHTNESS) / half);
  }
  analogWrite(pin, val);
}

static void breathing(uint8_t pin) {
  // 2s period broken into 100ms steps (20 steps total)
  static uint8_t step = 0;
  breathe_step(pin, step, 20);
  step = (step + 1) % 20;
  vTaskDelay(pdMS_TO_TICKS(100));
}

static void breathing_fast(uint8_t pin) {
  // 0.5s period broken into 100ms steps (5 steps total)
  static uint8_t step = 0;
  breathe_step(pin, step, 5);
  step = (step + 1) % 5;
  vTaskDelay(pdMS_TO_TICKS(100));
}

void LedStatus_Task(void *pvParameters) {
  Robot *robot = (Robot *)pvParameters;

  while (!led_initialized) {
    vTaskDelay(pdMS_TO_TICKS(10));
  }

  while (1) {
    switch (current_led_status) {
    case LED_STATUS_STARTUP:
      led_brightness = 127;
      breathing_fast(led_pin);
      break;

    case LED_STATUS_READY:
      led_brightness = 127;
      blink_pattern(1, 50, 950);
      break;

    case LED_STATUS_MOVING:
      led_brightness = 127;
      breathing(led_pin);
      break;

    case LED_STATUS_ERROR:
      led_brightness = 255;
      blink_pattern(1, 100, 100);
      break;

    case LED_STATUS_LOW_BATTERY:
      led_brightness = 50;
      blink_pattern(3, 100, 100);
      vTaskDelay(pdMS_TO_TICKS(1000));
      break;

    case LED_STATUS_BREATHING:
      led_brightness = 127;
      breathing(led_pin);
      break;

    case LED_STATUS_BREATHING_FAST:
      led_brightness = 127;
      breathing_fast(led_pin);
      break;

    case LED_STATUS_CHARGED:
      led_brightness = 255;
      analogWrite(led_pin, led_brightness);
      vTaskDelay(pdMS_TO_TICKS(100));
      break;

    default:
      analogWrite(led_pin, 0);
      vTaskDelay(pdMS_TO_TICKS(100));
      break;
    }
  }
}
