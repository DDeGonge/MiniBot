#ifndef QUEUE_STRUCTS_H
#define QUEUE_STRUCTS_H

#include "ESPNowMessages.h"
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

// Command type discriminator
enum CommandType {
  CMD_TYPE_GUI = 0,
  CMD_TYPE_MOT_TEST = 1,
  CMD_TYPE_POS_SYNC = 2
};

// Internal queue message types
struct GUICommand {
  uint8_t targetID;
  uint8_t requestType; // 0 = position command, 1 = position request, 2 = magnet
                       // request
  float x;
  float y;
  float angle;
  float duration;
};

// Union to allow different command types through the same queue
struct CommandMessage {
  uint8_t commandType; // Discriminator: 0=GUICommand, 1=MotTestCommand
  union {
    GUICommand guiCmd;
    MotTestCommand motCmd;
  } data;
};

struct GUIStatus {
  uint8_t targetID;
  bool ackReceived;
  float currentX;
  float currentY;
  float currentAngle;
  uint32_t timestamp;
  float batteryVoltage;
  float magnetX_gauss;
  float magnetY_gauss;
  float magnetZ_gauss;
  bool magnetFieldValid;
  uint8_t syncStatus = 0; // 0=no update, 1=ACK, 2=NACK
};

// Queue handles
extern QueueHandle_t commandQueue;      // All tasks -> Communicator
extern QueueHandle_t guiStatusQueue;    // Communicator -> GUI
extern QueueHandle_t pythonStatusQueue; // Communicator -> Python
extern QueueHandle_t i2cStatusQueue;    // Communicator -> I2C

// Broadcast status to all communication tasks
void broadcastStatus(const GUIStatus &status);

// Initialize queues
void initQueues();

#endif // QUEUE_STRUCTS_H
