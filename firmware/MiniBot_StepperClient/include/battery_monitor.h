#ifndef __BATTERY_MONITOR_H__
#define __BATTERY_MONITOR_H__

#include "robot.h"
#include "utils.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

/**
 * Parameters passed to BatteryMonitor_Task.
 * Holds the robot instance and handles for all tasks that should be
 * suspended when the battery condition requires it.
 */
typedef struct {
  Robot *robot;
  TaskHandle_t kinematics_task;
  TaskHandle_t communicator_task;
  TaskHandle_t position_estimator_sensor_task;
  TaskHandle_t position_estimator_calc_task;
} BatteryMonitorParams;

/**
 * Battery Monitor Task
 *
 * Priority: LOW
 * Responsible for:
 * - Periodically reading battery voltage via ADC
 * - Updating robot state with current voltage
 * - Detecting low battery conditions
 * - Triggering alerts when voltage drops below threshold
 *
 * @param pvParameters Pointer to initialization parameters (unused)
 */
void BatteryMonitor_Task(void *pvParameters);

/**
 * Initialize the battery monitor
 * Should be called before starting the task
 *
 * @param adc_pin GPIO pin connected to battery voltage divider
 * @param low_batt_threshold_v Voltage threshold for low battery detection
 * @return true on success, false on failure
 */
bool BatteryMonitor_Init(uint8_t adc_pin);

#endif // __BATTERY_MONITOR_H__
