#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

#define SERIAL_BAUD_RATE 921600

// System config flags
#define ENABLE_JOYSTICK_MODE  true
#define ENABLE_WEB_GUI        false
#define ENABLE_DEBUG_PRINTS   false

// Random GPIO
#define STATUS_LED_WHITE_PIN 5
#define STATUS_LED_RED_PIN 4

// Timing parameters (milliseconds)
#define EMAG_FRAME_LEN_MS            100    // Total frame length
#define EMAG_COUNT                   3      // Number of electromagnets in pattern
#define EMAG_FWD_ON_TIME_MS          7      // How long forward power is applied
#define EMAG_REV_ON_TIME_MS          7      // How long reverse power is applied

// PosSync burst configuration
#define POS_SYNC_INITIAL_DELAY_MS    5      // Delay before first sync pulse
#define POS_SYNC_BURST_COUNT         30     // Number of sync pulses in burst
#define POS_SYNC_BURST_INTERVAL_MS   3      // Interval between burst pulses

// GPIO pins for electromagnets (A=forward drive, B=reverse drive)
const uint8_t EMAG_PINS_A[EMAG_COUNT] = {
  6,
  8,
  18,
};
const uint8_t EMAG_PINS_B[EMAG_COUNT] = {
  7,
  9,
  19,
};

// ============= Joystick Configuration =============
#if ENABLE_JOYSTICK_MODE

#define JOYSTICK_TARGET_ID 0xFF  // Broadcast ID for all robots

#define JOYSTICK_THROTTLE_PIN 1
#define JOYSTICK_STEERING_PIN 3

#define JOYSTICK_THROTTLE_CENTER 2190
#define JOYSTICK_STEERING_CENTER 2220

#define JOYSTICK_THROTTLE_INVERT -1
#define JOYSTICK_STEERING_INVERT 1

#define JOYSTICK_DEADZONE 50
#define JOYSTICK_MIN_TICK 50
#define JOYSTICK_MAX_TICK 4050

#define JOYSTICK_MAX_MOTOR_VELOCITY 127   // Max input value (int8)
#define JOYSTICK_STEERING_SCALE 0.8       // sets upper limit for steering velocity as a fraction of max velocity

// Reduces steering authority with higher throttle
#define JOYSTICK_STEERING_INFLUENCE_FACTOR 0.70

#define JOYSTICK_UPDATE_INTERVAL_MS 100
#define JOYSTICK_WATCHDOG_INTERVAL_MS 900 // Ensure the watchdog isn't tripped (1s timeout)

#endif // ENABLE_JOYSTICK_MODE

#if ENABLE_DEBUG_PRINTS
  #define DEBUG_PRINTLN(x) Serial.println(x)
  #define DEBUG_PRINTF(fmt, ...) Serial.printf(fmt, ##__VA_ARGS__)
#else
  #define DEBUG_PRINTLN(x)
  #define DEBUG_PRINTF(fmt, ...)
#endif

#endif // CONFIG_H
