#include "QueueStructs.h"
#include "config.h"
#include <Arduino.h>

// Queue handle definitions
QueueHandle_t commandQueue = NULL;
QueueHandle_t guiStatusQueue = NULL;
QueueHandle_t pythonStatusQueue = NULL;
QueueHandle_t i2cStatusQueue = NULL;

void initQueues() {
  commandQueue = xQueueCreate(10, sizeof(CommandMessage));
  guiStatusQueue = xQueueCreate(20, sizeof(GUIStatus));
  pythonStatusQueue = xQueueCreate(20, sizeof(GUIStatus));
  i2cStatusQueue = xQueueCreate(20, sizeof(GUIStatus));

  if (commandQueue == NULL || guiStatusQueue == NULL ||
      pythonStatusQueue == NULL || i2cStatusQueue == NULL) {
    DEBUG_PRINTLN("Failed to create queues!");
  } else {
    DEBUG_PRINTLN("Queues created successfully");
  }
}

// Broadcast status to all communication tasks
void broadcastStatus(const GUIStatus &status) {
  xQueueSend(guiStatusQueue, &status, 0);
  xQueueSend(pythonStatusQueue, &status, 0);
  xQueueSend(i2cStatusQueue, &status, 0);
}
