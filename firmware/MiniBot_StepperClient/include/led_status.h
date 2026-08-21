#ifndef __LED_STATUS_H__
#define __LED_STATUS_H__

#include "robot.h"
#include <Arduino.h>

/**
 * System status indicators for LED display
 */
typedef enum {
  LED_STATUS_STARTUP = 0,
  LED_STATUS_READY = 1,
  LED_STATUS_MOVING = 2,
  LED_STATUS_ERROR = 3,
  LED_STATUS_LOW_BATTERY = 4,
  LED_STATUS_BREATHING = 5,
  LED_STATUS_BREATHING_FAST = 6,
  LED_STATUS_CHARGED = 7
} LedStatus;

/**
 * LED Status Indicator Task
 *
 * Priority: LOWEST
 * Responsible for:
 * - Monitoring robot system status
 * - Controlling status LED indicator
 * - Displaying system state via LED patterns (blink patterns, colors if RGB)
 * - Providing visual feedback for debugging and user indication
 *
 * @param pvParameters Pointer to initialization parameters (unused)
 */
void LedStatus_Task(void *pvParameters);

/**
 * Initialize the LED status indicator
 * Should be called before starting the task
 *
 * @param led_pin GPIO pin connected to status LED
 * @return true on success, false on failure
 */
bool LedStatus_Init(uint8_t led_pin);

/**
 * Set the current LED status
 * @param status LED status indicator
 */
void LedStatus_SetStatus(LedStatus status);

/**
 * Get the current LED status
 * @return Current LED status
 */
LedStatus LedStatus_GetStatus(void);

#endif // __LED_STATUS_H__
