#ifndef ESP_NOW_TASK_H
#define ESP_NOW_TASK_H

#include "ESPNowMessages.h"
#include <Arduino.h>
#include <esp_now.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Task handle
extern TaskHandle_t espNowTaskHandle;

// Peer MAC address
extern uint8_t broadcastAddress[6];

// Initialize ESP-NOW
void initESPNow();

// FreeRTOS ESP-NOW task
void espNowTask(void *parameter);

// ESP-NOW callbacks
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
void onDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len);

#endif // ESP_NOW_TASK_H
