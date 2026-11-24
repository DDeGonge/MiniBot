#ifndef __OTA_UPDATER_H__
#define __OTA_UPDATER_H__

#include "messages_espnow.h"
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

class Robot;

/**
 * OTA Updater Task
 *
 * Priority: LOW (blocks on a semaphore until a command arrives, then runs
 * the full WiFi-join + HTTP-download + flash-write sequence synchronously)
 *
 * Responsible for:
 * - Joining the Server's WiFi AP and pulling the staged firmware over HTTP
 * - Verifying size/CRC32, writing to the inactive OTA partition
 * - Pausing motion and other radio-sensitive tasks for the duration
 * - Reporting success/failure back to the Server via ESP-NOW
 * - Rebooting into the new image on success
 */
struct OtaUpdaterParams {
  Robot *robot;
  TaskHandle_t kinematics_task;
  TaskHandle_t communicator_task;
  TaskHandle_t position_estimator_sensor_task;
  TaskHandle_t position_estimator_calc_task;
};

/**
 * Initialize OTA updater resources (persisted version storage, command
 * handoff primitive). Must be called before starting the task.
 * @return true on success, false on failure
 */
bool OtaUpdater_Init();

/**
 * Called from the ESP-NOW receive path when an OtaCommand targeting this bot
 * arrives. Must be fast/non-blocking — it just captures the command and
 * wakes OtaUpdater_Task to do the actual work.
 *
 * @param mac_addr MAC address of the sender (the Server), used to reply
 * @param cmd The received OtaCommand
 */
void OtaUpdater_OnCommandReceived(const uint8_t *mac_addr,
                                 const OtaCommand *cmd);

/**
 * FreeRTOS OTA updater task entry point
 * @param pvParameters Pointer to an OtaUpdaterParams struct
 */
void OtaUpdater_Task(void *pvParameters);

#endif // __OTA_UPDATER_H__
