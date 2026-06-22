#ifndef GUI_TASK_H
#define GUI_TASK_H

#include "ESPNowMessages.h"
#include "QueueStructs.h"
#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Web server and WebSocket
extern AsyncWebServer server;
extern AsyncWebSocket ws;

// Status storage for 36 robots (9x4 grid)
extern GUIStatus robotStatus[36];

// Task handle
extern TaskHandle_t guiTaskHandle;

// Initialize GUI components
void initGUI();

// FreeRTOS GUI task
void guiTask(void *parameter);

// WebSocket event handler
void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
               AwsEventType type, void *arg, uint8_t *data, size_t len);

#endif // GUI_TASK_H
