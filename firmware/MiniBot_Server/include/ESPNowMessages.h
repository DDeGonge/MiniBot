#ifndef __MESSAGES_ESPNOW_H__
#define __MESSAGES_ESPNOW_H__

#include <stdint.h>

typedef enum {
  MSG_TYPE_MOT_TEST_COMMAND = 0,
  MSG_TYPE_POSITION_COMMAND = 1,
  MSG_TYPE_POSITION_REQUEST = 2,
  MSG_TYPE_ACK_MESSAGE = 3,
  MSG_TYPE_NACK_MESSAGE = 4,
  MSG_TYPE_MAG_REQUEST = 5,
  MSG_TYPE_MAG_FIELD_RESPONSE = 6,
  MSG_TYPE_POS_SYNC_COMMAND = 7,
  MSG_TYPE_POS_SYNC = 8,
  MSG_TYPE_OTA_COMMAND = 9,
  MSG_TYPE_OTA_RESULT = 10,
} EspNowMessageType;

typedef enum {
  ERR_INVALID_MSG_SIZE = 0,
  ERR_QUEUE_FULL = 1,
  ERR_ROBOT_UNAVAILABLE = 2,
  ERR_NOT_IMPLEMENTED = 3,
  ERR_UNKNOWN_MSG = 4,
  ERR_LOW_BATTERY = 5,
  ERR_SYNC_TIMEOUT = 6
} EspNowErrorType;

// Result codes reported by a bot after attempting a firmware update
typedef enum {
  OTA_RESULT_SUCCESS = 0,
  OTA_RESULT_ALREADY_CURRENT = 1,
  OTA_RESULT_WIFI_FAILED = 2,
  OTA_RESULT_DOWNLOAD_FAILED = 3,
  OTA_RESULT_CRC_MISMATCH = 4,
  OTA_RESULT_FLASH_FAILED = 5,
} OtaResultCode;

typedef struct __attribute__((packed)) {
  uint8_t targetID;
  uint8_t msg_type;
  bool enabled;
  int8_t m0_vel;
  int8_t m1_vel;
} MotTestCommand;

typedef struct __attribute__((packed)) {
  uint8_t targetID;
  uint8_t msg_type;
  uint32_t timestamp;
  float target_x_mm;
  float target_y_mm;
  float target_a_rad;
  float move_duration_ms;
} PositionCommand;

typedef struct __attribute__((packed)) {
  uint8_t targetID;
  uint8_t msg_type;
  uint32_t timestamp;
} PositionRequest;

typedef struct __attribute__((packed)) {
  uint8_t targetID;
  uint8_t msg_type;
  uint32_t timestamp;
  uint16_t timeout_ms;
} PosSyncCommand;

typedef struct __attribute__((packed)) {
  uint8_t targetID;
  uint8_t msg_type;
  uint32_t timestamp;
  uint32_t next_frame_us;
} PosSync;

typedef struct __attribute__((packed)) {
  uint8_t responderID;
  uint8_t msg_type;
  uint32_t timestamp;
  float x;
  float y;
  float orientation_rad;
  float battery_voltage;
} AckMessage;

typedef struct __attribute__((packed)) {
  uint8_t responderID;
  uint8_t msg_type;
  uint32_t timestamp;
  uint8_t err_type;
} NackMessage;

typedef struct __attribute__((packed)) {
  uint8_t targetID;
  uint8_t msg_type;
  uint32_t timestamp;
} MagneticFieldRequest;

typedef struct __attribute__((packed)) {
  uint8_t responderID;
  uint8_t msg_type;
  uint32_t timestamp;
  float field_x_gauss;
  float field_y_gauss;
  float field_z_gauss;
} MagneticFieldResponse;

// Sent server -> bot: a new firmware image is staged and ready to fetch
typedef struct __attribute__((packed)) {
  uint8_t targetID;
  uint8_t msg_type;
  uint32_t timestamp;
  uint32_t fw_version;
  uint32_t fw_size;
  uint32_t fw_crc32;
} OtaCommand;

// Sent bot -> server: outcome of an OTA attempt
typedef struct __attribute__((packed)) {
  uint8_t responderID;
  uint8_t msg_type;
  uint32_t timestamp;
  uint8_t result_code;
  uint32_t fw_version;
} OtaResult;

#endif // __MESSAGES_ESPNOW_H__