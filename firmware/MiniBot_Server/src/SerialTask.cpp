#include "SerialTask.h"
#include "ESPNowMessages.h"
#include "ElectromagnetTask.h"
#include "QueueStructs.h"
#include "config.h"
#include <Arduino.h>
#include <stdlib.h>
#include <string.h>

// Task handle
TaskHandle_t serialTaskHandle = NULL;

// Line accumulation buffer for inbound serial data
#define SERIAL_LINE_BUF_SIZE 128
static char s_lineBuf[SERIAL_LINE_BUF_SIZE];
static int s_lineLen = 0;

// ============================================================
// Helpers
// ============================================================

// Parse a target ID token that may be decimal ("5") or hex ("0x05")
static inline uint8_t parseID(const char *token) {
  return (uint8_t)strtol(token, NULL, 0);
}

// ============================================================
// Outbound: GUIStatus -> serial line
//
// Discrimination priority:
//   magnetFieldValid            -> >6 (MSG_TYPE_MAG_FIELD_RESPONSE)
//   syncStatus == 2 (NACK)      -> >4 (MSG_TYPE_NACK_MESSAGE), err_type =
//   ERR_SYNC_TIMEOUT = 6 syncStatus == 1 || ackReceived -> >3
//   (MSG_TYPE_ACK_MESSAGE) otherwise                   -> nothing (intermediate
//   broadcast with no new data)
//
// Writes the formatted line into buf (size bytes). Returns bytes written, or
// 0 if this status item produces no output.
// ============================================================
static int formatStatusInto(char *buf, int size, const GUIStatus &s) {
  if (s.magnetFieldValid) {
    return snprintf(buf, size, ">6,0x%02X,%.4f,%.4f,%.4f,%u\n", s.targetID,
                    s.magnetX_gauss, s.magnetY_gauss, s.magnetZ_gauss,
                    s.timestamp);
  } else if (s.syncStatus == 2) {
    return snprintf(buf, size, ">4,0x%02X,%u,%u\n", s.targetID,
                    (uint8_t)ERR_SYNC_TIMEOUT, s.timestamp);
  } else if (s.syncStatus == 1 || s.ackReceived) {
    return snprintf(buf, size, ">3,0x%02X,%.2f,%.2f,%.4f,%u,%.2f\n", s.targetID,
                    s.currentX, s.currentY, s.currentAngle, s.timestamp,
                    s.batteryVoltage);
  }
  return 0;
}

// ============================================================
// Inbound: parse and dispatch one null-terminated line
//
// Expected format: ><type_int>[,field,field,...]\n
//   type = EspNowMessageType value (0-7) or local extension:
//     254  -> EMAG toggle    >254,<0|1>
//     255  -> PING           >255
// ============================================================
static void processLine(char *line) {
  if (line[0] != '>')
    return;

  char *p = line + 1; // skip leading '>'

  char *typeToken = strtok(p, ",\r\n");
  if (typeToken == NULL || typeToken[0] == '\0') {
    Serial.println(">ERR,empty message");
    return;
  }

  char *endptr;
  long msgType = strtol(typeToken, &endptr, 0);
  if (endptr == typeToken) {
    Serial.printf(">ERR,bad type: %s\n", typeToken);
    return;
  }

  switch ((int)msgType) {

  // --------------------------------------------------------
  // 0: MSG_TYPE_MOT_TEST_COMMAND
  // Format: >0,<id>,<enable>,<m0_vel>,<m1_vel>
  // --------------------------------------------------------
  case MSG_TYPE_MOT_TEST_COMMAND: {
    char *idStr = strtok(NULL, ",\r\n");
    char *enStr = strtok(NULL, ",\r\n");
    char *m0Str = strtok(NULL, ",\r\n");
    char *m1Str = strtok(NULL, ",\r\n");
    if (!idStr || !enStr || !m0Str || !m1Str) {
      Serial.println(">ERR,MOT: need id,enable,m0,m1");
      return;
    }
    CommandMessage msg = {};
    msg.commandType = CMD_TYPE_MOT_TEST;
    MotTestCommand &mot = msg.data.motCmd;
    mot.targetID = parseID(idStr);
    mot.msg_type = MSG_TYPE_MOT_TEST_COMMAND;
    mot.enabled = (atoi(enStr) != 0);
    mot.m0_vel = (int8_t)atoi(m0Str);
    mot.m1_vel = (int8_t)atoi(m1Str);
    if (xQueueSend(commandQueue, &msg, 0) != pdPASS) {
      Serial.println(">ERR,queue full");
    }
    break;
  }

  // --------------------------------------------------------
  // 1: MSG_TYPE_POSITION_COMMAND
  // Format: >1,<id>,<x_mm>,<y_mm>,<a_rad>,<dur_ms>
  // --------------------------------------------------------
  case MSG_TYPE_POSITION_COMMAND: {
    char *idStr = strtok(NULL, ",\r\n");
    char *xStr = strtok(NULL, ",\r\n");
    char *yStr = strtok(NULL, ",\r\n");
    char *aStr = strtok(NULL, ",\r\n");
    char *dStr = strtok(NULL, ",\r\n");
    if (!idStr || !xStr || !yStr || !aStr || !dStr) {
      Serial.println(">ERR,CMD: need id,x,y,angle,dur");
      return;
    }
    CommandMessage msg = {};
    msg.commandType = CMD_TYPE_GUI;
    GUICommand &cmd = msg.data.guiCmd;
    cmd.requestType = 0; // position command
    cmd.targetID = parseID(idStr);
    cmd.x = atof(xStr);
    cmd.y = atof(yStr);
    cmd.angle = atof(aStr);
    cmd.duration = atof(dStr);
    if (xQueueSend(commandQueue, &msg, 0) != pdPASS) {
      Serial.println(">ERR,queue full");
    }
    break;
  }

  // --------------------------------------------------------
  // 2: MSG_TYPE_POSITION_REQUEST
  // Format: >2,<id>
  // --------------------------------------------------------
  case MSG_TYPE_POSITION_REQUEST: {
    char *idStr = strtok(NULL, ",\r\n");
    if (!idStr) {
      Serial.println(">ERR,REQ: need id");
      return;
    }
    CommandMessage msg = {};
    msg.commandType = CMD_TYPE_GUI;
    GUICommand &cmd = msg.data.guiCmd;
    cmd.requestType = 1; // position request
    cmd.targetID = parseID(idStr);
    if (xQueueSend(commandQueue, &msg, 0) != pdPASS) {
      Serial.println(">ERR,queue full");
    }
    break;
  }

  // --------------------------------------------------------
  // 5: MSG_TYPE_MAG_REQUEST
  // Format: >5,<id>
  // --------------------------------------------------------
  case MSG_TYPE_MAG_REQUEST: {
    char *idStr = strtok(NULL, ",\r\n");
    if (!idStr) {
      Serial.println(">ERR,MAG: need id");
      return;
    }
    CommandMessage msg = {};
    msg.commandType = CMD_TYPE_GUI;
    GUICommand &cmd = msg.data.guiCmd;
    cmd.requestType = 2; // magnetic field request
    cmd.targetID = parseID(idStr);
    if (xQueueSend(commandQueue, &msg, 0) != pdPASS) {
      Serial.println(">ERR,queue full");
    }
    break;
  }

  // --------------------------------------------------------
  // 7: MSG_TYPE_POS_SYNC_COMMAND
  // Format: >7
  // --------------------------------------------------------
  case MSG_TYPE_POS_SYNC_COMMAND: {
    CommandMessage msg = {};
    msg.commandType = CMD_TYPE_POS_SYNC;
    if (xQueueSend(commandQueue, &msg, 0) != pdPASS) {
      Serial.println(">ERR,queue full");
    }
    break;
  }

  // --------------------------------------------------------
  // 254: Local - Electromagnet enable/disable
  // Format: >254,<0|1>
  // --------------------------------------------------------
  case 254: {
    char *enStr = strtok(NULL, ",\r\n");
    if (!enStr) {
      Serial.println(">ERR,EMAG: need 0 or 1");
      return;
    }
    setElectromagnetEnabled(atoi(enStr) != 0);
    break;
  }

  // --------------------------------------------------------
  // 255: Local - PING
  // Format: >255  ->  reply: >255
  // --------------------------------------------------------
  case 255: {
    Serial.println(">255");
    break;
  }

  default:
    Serial.printf(">ERR,unknown type: %ld\n", msgType);
    break;
  }
}

// ============================================================
// Public API
// ============================================================

void initSerial() {
  Serial.setTxBufferSize(1024);
  Serial.begin(SERIAL_BAUD_RATE);
}

void serialTask(void *parameter) {
  for (;;) {
    // --- Outbound: drain pythonStatusQueue, batch into one buffer, single
    // write ---
    {
      static char txBatch[512];
      int batchLen = 0;
      GUIStatus status;
      while (xQueueReceive(pythonStatusQueue, &status, 0) == pdPASS) {
        int remaining = (int)sizeof(txBatch) - batchLen;
        if (remaining <= 1)
          break; // batch full; remainder stays in queue for next cycle
        int n = formatStatusInto(txBatch + batchLen, remaining, status);
        if (n > 0 && n < remaining) {
          batchLen += n;
        }
      }
      if (batchLen > 0) {
        Serial.write((const uint8_t *)txBatch, batchLen);
      }
    }

    // --- Inbound: accumulate serial bytes, dispatch on newline ---
    while (Serial.available() > 0) {
      char c = (char)Serial.read();
      if (c == '\n') {
        if (s_lineLen > 0) {
          s_lineBuf[s_lineLen] = '\0';
          processLine(s_lineBuf);
          s_lineLen = 0;
        }
      } else if (c != '\r') {
        if (s_lineLen < SERIAL_LINE_BUF_SIZE - 1) {
          s_lineBuf[s_lineLen++] = c;
        } else {
          // Buffer overflow: discard and report
          s_lineLen = 0;
          Serial.println(">ERR,line too long");
        }
      }
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}
