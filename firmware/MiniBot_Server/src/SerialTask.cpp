#include "SerialTask.h"
#include "ESPNowMessages.h"
#include "ElectromagnetTask.h"
#include "QueueStructs.h"
#include "config.h"
#include <Arduino.h>
#include <LittleFS.h>
#include <stdlib.h>
#include <string.h>

// Task handle
TaskHandle_t serialTaskHandle = NULL;

// Line accumulation buffer for inbound serial data
#define SERIAL_LINE_BUF_SIZE 128
static char s_lineBuf[SERIAL_LINE_BUF_SIZE];
static int s_lineLen = 0;

// ============================================================
// OTA firmware upload (raw binary sub-protocol over the same UART)
//
// >200,{size},{crc32}   Host -> ESP: begin upload, reply >200,READY
//                       then host streams exactly {size} raw bytes
// >200,OK | >200,ERR,{msg}   ESP -> Host: upload result
// >201,{id},{version}  Host -> ESP: push the staged firmware to a bot
// ============================================================
static bool s_otaUploadActive = false;
static uint32_t s_otaExpectedSize = 0;
static uint32_t s_otaExpectedCrc32 = 0;
static uint32_t s_otaBytesReceived = 0;
static uint32_t s_otaRunningCrc32 = 0xFFFFFFFFU;
static uint32_t s_otaLastByteMs = 0;
static uint32_t s_otaStagedCrc32 = 0;
static File s_otaFile;

static uint32_t s_crc32Table[256];
static bool s_crc32TableReady = false;

static void initCrc32Table() {
  for (uint32_t i = 0; i < 256; i++) {
    uint32_t c = i;
    for (int k = 0; k < 8; k++) {
      c = (c & 1) ? (0xEDB88320U ^ (c >> 1)) : (c >> 1);
    }
    s_crc32Table[i] = c;
  }
  s_crc32TableReady = true;
}

// Standard CRC-32 (zlib/IEEE 802.3), matching Python's zlib.crc32()
static uint32_t crc32Update(uint32_t crc, const uint8_t *buf, size_t len) {
  if (!s_crc32TableReady) {
    initCrc32Table();
  }
  for (size_t i = 0; i < len; i++) {
    crc = s_crc32Table[(crc ^ buf[i]) & 0xFF] ^ (crc >> 8);
  }
  return crc;
}

static void abortOtaUpload(const char *reason) {
  if (s_otaFile) {
    s_otaFile.close();
  }
  LittleFS.remove(OTA_FIRMWARE_PATH);
  s_otaUploadActive = false;
  Serial.printf(">200,ERR,%s\n", reason);
}

// Called every serialTask loop iteration while an upload is in progress.
// Drains whatever raw bytes are CURRENTLY buffered (not just one fixed-size
// chunk) so nothing is left sitting in the small UART ring buffer across
// task-loop iterations — at 921600 baud a 512-byte cap left the receiver
// unable to keep up with the sender, causing silent RX overflow/data loss
// and an eventual false "timeout" even though the host had sent everything.
static void pumpOtaUpload() {
  int available = Serial.available();
  if (available <= 0) {
    if ((millis() - s_otaLastByteMs) > OTA_UPLOAD_STALL_TIMEOUT_MS) {
      abortOtaUpload("timeout");
    }
    return;
  }

  uint8_t chunk[OTA_UPLOAD_CHUNK_SIZE];
  while (available > 0 && s_otaBytesReceived < s_otaExpectedSize) {
    uint32_t remaining = s_otaExpectedSize - s_otaBytesReceived;
    size_t toRead = (size_t)available;
    if (toRead > OTA_UPLOAD_CHUNK_SIZE) {
      toRead = OTA_UPLOAD_CHUNK_SIZE;
    }
    if (toRead > remaining) {
      toRead = remaining;
    }

    // toRead never exceeds what's already buffered, so this returns
    // immediately instead of blocking on the Stream read timeout.
    int n = Serial.readBytes(chunk, toRead);
    if (n <= 0) {
      break;
    }

    s_otaFile.write(chunk, n);
    s_otaRunningCrc32 = crc32Update(s_otaRunningCrc32, chunk, n);
    s_otaBytesReceived += n;
    s_otaLastByteMs = millis();
    available = Serial.available();
  }

  if (s_otaBytesReceived >= s_otaExpectedSize) {
    s_otaFile.close();
    uint32_t finalCrc = s_otaRunningCrc32 ^ 0xFFFFFFFFU;
    if (finalCrc == s_otaExpectedCrc32) {
      s_otaStagedCrc32 = finalCrc;
      Serial.println(">200,OK");
    } else {
      LittleFS.remove(OTA_FIRMWARE_PATH);
      Serial.printf(">200,ERR,crc mismatch got 0x%08X want 0x%08X\n", finalCrc,
                   s_otaExpectedCrc32);
    }
    s_otaUploadActive = false;
  }
}

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
//   otaResultValid              -> >202 (MSG_TYPE_OTA_RESULT)
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
  if (s.otaResultValid) {
    return snprintf(buf, size, ">202,0x%02X,%d,%u,%u\n", s.targetID,
                    s.otaSuccess ? 1 : 0, (unsigned)s.otaErrCode,
                    (unsigned)s.otaVersion);
  } else if (s.magnetFieldValid) {
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
  // 200: OTA upload begin
  // Format: >200,{size},{crc32}  ->  reply: >200,READY
  // Host then streams {size} raw bytes; final reply: >200,OK / >200,ERR,...
  // --------------------------------------------------------
  case 200: {
    char *sizeStr = strtok(NULL, ",\r\n");
    char *crcStr = strtok(NULL, ",\r\n");
    if (!sizeStr || !crcStr) {
      Serial.println(">200,ERR,need size,crc32");
      return;
    }
    if (s_otaUploadActive) {
      Serial.println(">200,ERR,upload already in progress");
      return;
    }

    uint32_t expectedSize = (uint32_t)strtoul(sizeStr, NULL, 0);
    uint32_t expectedCrc32 = (uint32_t)strtoul(crcStr, NULL, 0);
    if (expectedSize == 0 || expectedSize > LittleFS.totalBytes()) {
      Serial.println(">200,ERR,bad size");
      return;
    }

    if (LittleFS.exists(OTA_FIRMWARE_PATH)) {
      LittleFS.remove(OTA_FIRMWARE_PATH);
    }
    s_otaFile = LittleFS.open(OTA_FIRMWARE_PATH, FILE_WRITE);
    if (!s_otaFile) {
      Serial.println(">200,ERR,failed to open storage");
      return;
    }

    s_otaExpectedSize = expectedSize;
    s_otaExpectedCrc32 = expectedCrc32;
    s_otaBytesReceived = 0;
    s_otaRunningCrc32 = 0xFFFFFFFFU;
    s_otaLastByteMs = millis();
    s_otaUploadActive = true;
    Serial.println(">200,READY");
    break;
  }

  // --------------------------------------------------------
  // 201: OTA start - push the currently staged firmware to a bot
  // Format: >201,{id},{version}
  // --------------------------------------------------------
  case 201: {
    char *idStr = strtok(NULL, ",\r\n");
    char *verStr = strtok(NULL, ",\r\n");
    if (!idStr || !verStr) {
      Serial.println(">ERR,OTA: need id,version");
      return;
    }
    if (s_otaUploadActive || !LittleFS.exists(OTA_FIRMWARE_PATH)) {
      Serial.println(">ERR,OTA: no firmware staged");
      return;
    }

    File fw = LittleFS.open(OTA_FIRMWARE_PATH, FILE_READ);
    uint32_t fwSize = fw ? fw.size() : 0;
    if (fw) {
      fw.close();
    }

    CommandMessage msg = {};
    msg.commandType = CMD_TYPE_OTA;
    OtaCommand &ota = msg.data.otaCmd;
    ota.targetID = parseID(idStr);
    ota.msg_type = MSG_TYPE_OTA_COMMAND;
    ota.fw_version = (uint32_t)strtoul(verStr, NULL, 0);
    ota.fw_size = fwSize;
    ota.fw_crc32 = s_otaStagedCrc32;
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
  // Default RX ring buffer (256B) can't keep up with a bulk OTA transfer at
  // 921600 baud between task-loop iterations; enlarge it before begin().
  Serial.setRxBufferSize(4096);
  Serial.setTxBufferSize(1024);
  Serial.begin(SERIAL_BAUD_RATE);

  if (!LittleFS.begin(true)) {
    // Always print (not DEBUG_PRINTLN-gated) — this is critical for
    // diagnosing OTA storage failures, and ENABLE_DEBUG_PRINTS is normally
    // off in production builds.
    Serial.println(">ERR,LittleFS mount failed");
  } else {
    Serial.printf(">INFO,LittleFS mounted (%u/%u bytes used)\n",
                 (unsigned)LittleFS.usedBytes(), (unsigned)LittleFS.totalBytes());
  }
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
    // While an OTA upload is in progress, incoming bytes are the raw firmware
    // image itself, not text lines, so they bypass the line parser entirely.
    if (s_otaUploadActive) {
      pumpOtaUpload();
    } else {
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
    }

    // Shorter idle delay while an OTA upload is active, so the small UART
    // ring buffer can't fill up between drains.
    vTaskDelay(pdMS_TO_TICKS(s_otaUploadActive ? 1 : 10));
  }
}
