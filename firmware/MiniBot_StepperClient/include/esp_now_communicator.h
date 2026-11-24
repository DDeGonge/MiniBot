#ifndef __ESP_NOW_COMMUNICATOR_H__
#define __ESP_NOW_COMMUNICATOR_H__

#include "config.h"
#include "messages_espnow.h"
#include "motion_queue.h"
#include "motor_test_queue.h"
#include "robot.h"
#include <Arduino.h>
#include <WiFi.h>
#include <esp_mac.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <functional>
#include <vector>

// Callback type for received broadcast messages
typedef std::function<void(const uint8_t *mac_addr, const uint8_t *data,
                           int len)>
    EspNowReceiveCallback;

/**
 * ESP-NOW Communicator Task
 *
 * Priority: HIGH
 * Responsible for:
 * - Receiving commands via ESP-NOW protocol
 * - Parsing received motion and motor test commands
 * - Enqueuing commands to the motion and motor test queues
 * - Sending status updates back to the remote controller
 *
 * @param pvParameters Pointer to initialization parameters (unused)
 */
void EspNowCommunicator_Task(void *pvParameters);

/**
 * Initialize the ESP-NOW communicator
 * Should be called before starting the task
 *
 * @param motion_queue MotionQueue handle
 * @param motor_test_queue MotorTestQueue handle
 * @return true on success, false on failure
 */
bool EspNowCommunicator_Init(MotionQueue motion_queue,
                             MotorTestQueue motor_test_queue);

/**
 * Register a callback function to be called when broadcast messages are
 * received
 *
 * @param callback Function to call with (mac_addr, data, length)
 * @return true on success, false on failure
 */
bool EspNowCommunicator_RegisterCallback(EspNowReceiveCallback callback);

/**
 * Send an alert NACK message to the last known sender with the specified error
 * type Can be used for low battery, critical systems errors, or any other alert
 * condition
 *
 * @param error_type The error type to send (from EspNowErrorType enum or custom
 * values)
 * @return true on success, false on failure
 */
bool EspNowCommunicator_SendAlert(uint8_t error_type);

/**
 * Send an OtaResult message to a specific peer (the sender of the OtaCommand
 * being responded to)
 *
 * @param mac_addr MAC address of the recipient (the Server)
 * @param result_code One of the OtaResultCode enum values
 * @param fw_version Firmware version associated with the result
 * @return true on success, false on failure
 */
bool EspNowCommunicator_SendOtaResult(const uint8_t *mac_addr,
                                      uint8_t result_code,
                                      uint32_t fw_version);

/**
 * Force-reapply the ESP-NOW radio wake window / wake interval to match the
 * current duty-cycle state.
 *
 * set_duty_cycle() short-circuits when its tracked state matches the request,
 * but a WiFi.begin + WiFi.disconnect sequence (OTA joining the server's AP)
 * resets the ESP-IDF wake settings behind its back. After an OTA attempt the
 * communicator task's next set_duty_cycle call early-returns and the radio
 * stays mis-tuned for the rest of the boot. Call this after an OTA to put
 * the ESP-IDF wake configuration back in sync with duty_cycle_active.
 */
void EspNowCommunicator_RestoreDutyCycle();

#endif // __ESP_NOW_COMMUNICATOR_H__
