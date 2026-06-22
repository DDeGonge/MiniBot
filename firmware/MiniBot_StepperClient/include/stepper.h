#ifndef __STEPPER_H__
#define __STEPPER_H__

#include "esp_timer.h"
#include <Arduino.h>
#include <stdint.h>

class StepperDriver {
public:
  // Pin configuration
  uint8_t step_pin;
  uint8_t dir_pin;
  uint8_t enable_pin;
  uint8_t reset_pin;

private:
  // State
  bool enabled;
  int8_t current_direction; // 1 for forward, -1 for reverse
  volatile int32_t current_step_count;

  bool reverse_motor; // If true, reverses the direction signal
  bool timer_running = false;
  bool timer_initialized = false;
  uint32_t step_interval_us = 0;
  int64_t timer_start_time_us = 0;
  uint32_t timer_steps_target = 0;
  uint32_t timer_steps_taken = 0;

  esp_timer_handle_t s_step_timer = NULL;

public:
  /**
   * Initialize stepper driver with GPIO pins
   * @param step GPIO pin for step signal
   * @param dir GPIO pin for direction signal
   * @param enable GPIO pin for enable signal
   * @param reset GPIO pin for reset signal
   * @param reverse If true, reverse the motor direction
   * @return true on success, false on failure
   */
  bool initialize(uint8_t step, uint8_t dir, uint8_t enable, uint8_t reset,
                  bool reverse = false);

  /**
   * Set microstepping mode. Requires driver reset after setting.
   * Remember the reset and en pins are shared, so set both drivers
   * first before resetting.
   * @param step_lvl Logic level for STEP pin
   * @param dir_lvl Logic level for DIR pin
   * @return true on success, false on failure
   */
  bool setMicrostepping(bool step_lvl, bool dir_lvl);

  /**
   * Reset the stepper driver
   * @return true on success, false on failure
   */
  bool resetDriver();

  /**
   * Enable the stepper driver
   * @return true on success, false on failure
   */
  bool enable();

  /**
   * Disable the stepper driver
   * @return true on success, false on failure
   */
  bool disable();

  /**
   * Set stepper direction
   * @param direction true for forward, false for reverse
   * @return true on success, false on failure
   */
  bool setDirection(bool direction);

  /**
   * Generate a single step pulse
   * @return true on success, false on failure
   */
  bool step();

  /**
   * Generate a single step pulse from an ISR context.
   */
  static void stepISR(void *arg);

  /**
   * Initialize the step timer. Must be called before configureTimer or
   * startTimer.
   * @return true on success, false on failure
   */
  bool initTimer();

  /**
   * Configure timer for a specific velocity
   * @param velocity_rad_s Desired velocity in rad/s (used to calculate step
   * interval)
   * @param steps_per_rad Steps per radian for the wheel (used to calculate step
   * interval)
   * @param steps_to_take Total steps to take before stopping timer (0 for
   * continuous)
   * @return true on success, false on failure
   */
  bool configureTimer(float velocity_rad_s, float steps_per_rad,
                      int steps_to_take);

  /**
   * Start timer for continuous stepping
   * @return true on success, false if not initialize or already running
   */
  bool startTimer();

  /**
   * Stop timer
   * @return true on success, false if not initialized (but then how did it
   * start????)
   */
  bool stopTimer();

  bool isTimerRunning() const { return timer_running; }
};

#endif // __STEPPER_H__