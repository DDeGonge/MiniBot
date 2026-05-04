#include "CommunicatorTask.h"
#include "QueueStructs.h"
#include "ElectromagnetTask.h"
#include "config.h"
#include <esp_timer.h>

// Task handle
TaskHandle_t commTaskHandle = NULL;

// Store peer MAC addresses (broadcast by default)
uint8_t broadcastAddress[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// ACK tracking
static QueueHandle_t ackQueue = NULL;
static QueueHandle_t nackQueue = NULL;
static QueueHandle_t magFieldQueue = NULL;

// ESP-NOW callback when data is sent
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  DEBUG_PRINTLN("Broadcast Send Status: ");
  DEBUG_PRINTLN(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

// ESP-NOW callback when data is received
void onDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
  DEBUG_PRINTF("ESP-NOW data received: %d bytes\n", len);
  
  // Try to handle as AckMessage first
  if (len == sizeof(AckMessage)) {
    AckMessage ack;
    memcpy(&ack, incomingData, sizeof(ack));
    
    DEBUG_PRINTF("Received message - Type: %d, ResponderID: 0x%02X\n", ack.msg_type, ack.responderID);
    
    // Check if it's an ACK message
    if (ack.msg_type == MSG_TYPE_ACK_MESSAGE) {
      DEBUG_PRINTF("ACK message detected from robot 0x%02X\n", ack.responderID);
      DEBUG_PRINTF("  Position: (%.2f, %.2f) Angle: %.3f rad\n", ack.x, ack.y, ack.orientation_rad);
      DEBUG_PRINTF("  Timestamp: %u Battery: %.2fV\n", ack.timestamp, ack.battery_voltage);
      
      // Send to ACK queue for timeout handling
      if (ackQueue != NULL) {
        if (xQueueSend(ackQueue, &ack, 0) == pdPASS) {
          DEBUG_PRINTLN("  -> Sent to ACK queue");
        } else {
          DEBUG_PRINTLN("  -> ACK queue full!");
        }
      } else {
        DEBUG_PRINTLN("  -> ACK queue is NULL!");
      }
      
      // Also send to all status queues for GUI/Python/I2C update
      GUIStatus status;
      status.targetID = ack.responderID;
      status.ackReceived = true;
      status.currentX = ack.x;
      status.currentY = ack.y;
      status.currentAngle = ack.orientation_rad;
      status.timestamp = ack.timestamp;
      status.batteryVoltage = ack.battery_voltage;
      status.magnetFieldValid = false;
      
      broadcastStatus(status);
      DEBUG_PRINTLN("  -> Broadcast to all status queues");
      return;
    }
  }
  
  // Try to handle as MagneticFieldResponse
  if (len == sizeof(MagneticFieldResponse)) {
    MagneticFieldResponse magField;
    memcpy(&magField, incomingData, sizeof(magField));
    
    DEBUG_PRINTF("Received message - Type: %d, ResponderID: 0x%02X\n", magField.msg_type, magField.responderID);
    
    if (magField.msg_type == MSG_TYPE_MAG_REQUEST) {
      DEBUG_PRINTF("Magnetic field response detected from robot 0x%02X\n", magField.responderID);
      DEBUG_PRINTF("  Field: [%.2f, %.2f, %.2f] gauss\n", magField.field_x_gauss, magField.field_y_gauss, magField.field_z_gauss);
      DEBUG_PRINTF("  Timestamp: %u\n", magField.timestamp);
      
      // Send to magField queue for timeout handling
      if (magFieldQueue != NULL) {
        if (xQueueSend(magFieldQueue, &magField, 0) == pdPASS) {
          DEBUG_PRINTLN("  -> Sent to magField queue");
        } else {
          DEBUG_PRINTLN("  -> magField queue full!");
        }
      } else {
        DEBUG_PRINTLN("  -> magField queue is NULL!");
      }
      return;
    }
  }

  // Try to handle as NackMessage
  if (len == sizeof(NackMessage)) {
    NackMessage nack;
    memcpy(&nack, incomingData, sizeof(nack));
    if (nack.msg_type == MSG_TYPE_NACK_MESSAGE) {
      DEBUG_PRINTF("NACK received from robot 0x%02X, err_type=%d\n", nack.responderID, nack.err_type);
      if (nackQueue != NULL) {
        if (xQueueSend(nackQueue, &nack, 0) != pdPASS) {
          DEBUG_PRINTLN("  -> NACK queue full!");
        }
      }
      return;
    }
  }

  DEBUG_PRINTF("Warning: Unhandled message type or size mismatch (len=%d, AckMsg=%d, NackMsg=%d, MagField=%d)\n",
               len, sizeof(AckMessage), sizeof(NackMessage), sizeof(MagneticFieldResponse));
}

// Initialize ESP-NOW
void initESPNow() {
  // Create ACK queue
  ackQueue = xQueueCreate(10, sizeof(AckMessage));
  nackQueue = xQueueCreate(10, sizeof(NackMessage));
  magFieldQueue = xQueueCreate(10, sizeof(MagneticFieldResponse));
  
  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    DEBUG_PRINTLN("ESP-NOW initialization failed");
    return;
  }
  
  DEBUG_PRINTLN("ESP-NOW initialized");
  
  // Register callbacks
  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataRecv);
  
  // Add broadcast peer
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    DEBUG_PRINTLN("Failed to add peer");
  } else {
    DEBUG_PRINTLN("Broadcast peer added successfully");
  }
}

// FreeRTOS Communicator Task
void communicatorTask(void *parameter) {
  DEBUG_PRINTLN("Communicator Task started");
  
  CommandMessage msg;
  
  while (1) {
    // Check for commands from GUI or Joystick
    if (xQueueReceive(commandQueue, &msg, pdMS_TO_TICKS(10)) == pdPASS) {
      if (msg.commandType == CMD_TYPE_MOT_TEST) {
        // Handle MotTestCommand (from Joystick or other sources)
        MotTestCommand *motCmd = &msg.data.motCmd;
        Serial.printf("[COMM] Dequeued MotTestCmd: target=0x%02X enabled=%d M0=%d M1=%d\n",
                      motCmd->targetID, motCmd->enabled, motCmd->m0_vel, motCmd->m1_vel);
        
        // Send broadcast
        esp_err_t result = esp_now_send(broadcastAddress, (uint8_t*)motCmd, sizeof(MotTestCommand));
        
        if (result == ESP_OK) {
          Serial.println("[COMM] MotTestCommand broadcast OK");
        } else {
          Serial.printf("[COMM] MotTestCommand send FAILED: err=%d\n", result);
        }
      } else if (msg.commandType == CMD_TYPE_GUI) {
        // Handle GUICommand (from GUI)
        GUICommand *cmd = &msg.data.guiCmd;
      
        if (cmd->requestType == 1) {
          // Send position request
          PositionRequest req;
          req.targetID = cmd->targetID;
          req.msg_type = MSG_TYPE_POSITION_REQUEST;
          req.timestamp = millis();
          
          DEBUG_PRINTF("Sending position request to robot 0x%02X\n", cmd->targetID);
          
          // Clear any pending ACKs for this target
          AckMessage tempAck;
          while (xQueueReceive(ackQueue, &tempAck, 0) == pdPASS) {
            // Clear queue
          }
          
          // Send broadcast
          esp_err_t result = esp_now_send(broadcastAddress, (uint8_t*)&req, sizeof(req));
          
          if (result == ESP_OK) {
            DEBUG_PRINTLN("Position request broadcast sent");

            AckMessage ack;
            uint32_t startTime = millis();

            if (cmd->targetID == 0xFF) {
              // Broadcast request: collect all responses during the full window
              while ((millis() - startTime) < 500) {
                if (xQueueReceive(ackQueue, &ack, pdMS_TO_TICKS(10)) == pdPASS) {
                  if (ack.msg_type == MSG_TYPE_ACK_MESSAGE) {
                    DEBUG_PRINTF("ACK received from robot 0x%02X (broadcast req)\n", ack.responderID);
                    GUIStatus status;
                    status.targetID = ack.responderID;
                    status.ackReceived = true;
                    status.currentX = ack.x;
                    status.currentY = ack.y;
                    status.currentAngle = ack.orientation_rad;
                    status.timestamp = ack.timestamp;
                    status.batteryVoltage = ack.battery_voltage;
                    status.magnetFieldValid = false;
                    broadcastStatus(status);
                  }
                }
              }
            } else {
              // Unicast request: wait for the specific robot's ACK
              bool ackReceived = false;
              while ((millis() - startTime) < 500) {
                if (xQueueReceive(ackQueue, &ack, pdMS_TO_TICKS(10)) == pdPASS) {
                  if (ack.responderID == cmd->targetID && ack.msg_type == MSG_TYPE_ACK_MESSAGE) {
                    ackReceived = true;
                    DEBUG_PRINTF("ACK received from robot 0x%02X\n", cmd->targetID);
                    break;
                  }
                }
              }
              if (!ackReceived) {
                DEBUG_PRINTF("No ACK received from robot 0x%02X (timeout)\n", cmd->targetID);
                GUIStatus status;
                status.targetID = cmd->targetID;
                status.ackReceived = false;
                status.currentX = 0;
                status.currentY = 0;
                status.currentAngle = 0;
                status.timestamp = 0;
                status.batteryVoltage = 0;
                status.magnetFieldValid = false;
                broadcastStatus(status);
              }
            }
        } else {
          DEBUG_PRINTF("Error sending position request: %d\n", result);
        }
        
      } else if (cmd->requestType == 2) {
        // Send magnetic field request
        MagneticFieldRequest magReq;
        magReq.targetID = cmd->targetID;
        magReq.msg_type = MSG_TYPE_MAG_REQUEST;
        magReq.timestamp = millis();
        
        DEBUG_PRINTF("Sending magnetic field request to robot 0x%02X\n", cmd->targetID);
        
        // Clear any pending magField responses for this target
        MagneticFieldResponse tempMag;
        while (xQueueReceive(magFieldQueue, &tempMag, 0) == pdPASS) {
          // Clear queue
        }
        
        // Send broadcast
        esp_err_t result = esp_now_send(broadcastAddress, (uint8_t*)&magReq, sizeof(magReq));
        
        if (result == ESP_OK) {
          DEBUG_PRINTLN("Magnetic field request broadcast sent");

          MagneticFieldResponse magResp;
          uint32_t startTime = millis();

          if (cmd->targetID == 0xFF) {
            // Broadcast request: collect all responses during the full window
            while ((millis() - startTime) < 500) {
              if (xQueueReceive(magFieldQueue, &magResp, pdMS_TO_TICKS(10)) == pdPASS) {
                if (magResp.msg_type == MSG_TYPE_MAG_FIELD_RESPONSE) {
                  DEBUG_PRINTF("Magnetic field response from robot 0x%02X (broadcast req)\n", magResp.responderID);
                  GUIStatus status;
                  status.targetID = magResp.responderID;
                  status.ackReceived = true;
                  status.currentX = 0;
                  status.currentY = 0;
                  status.currentAngle = 0;
                  status.timestamp = magResp.timestamp;
                  status.batteryVoltage = -1.0f;
                  status.magnetX_gauss = magResp.field_x_gauss;
                  status.magnetY_gauss = magResp.field_y_gauss;
                  status.magnetZ_gauss = magResp.field_z_gauss;
                  status.magnetFieldValid = true;
                  broadcastStatus(status);
                }
              }
            }
          } else {
            // Unicast request: wait for the specific robot's response
            bool respReceived = false;
            while ((millis() - startTime) < 500) {
              if (xQueueReceive(magFieldQueue, &magResp, pdMS_TO_TICKS(10)) == pdPASS) {
                if (magResp.responderID == cmd->targetID && magResp.msg_type == MSG_TYPE_MAG_FIELD_RESPONSE) {
                  respReceived = true;
                  DEBUG_PRINTF("Magnetic field response received from robot 0x%02X\n", cmd->targetID);
                  GUIStatus status;
                  status.targetID = cmd->targetID;
                  status.ackReceived = true;
                  status.currentX = 0;
                  status.currentY = 0;
                  status.currentAngle = 0;
                  status.timestamp = magResp.timestamp;
                  status.batteryVoltage = -1.0f;
                  status.magnetX_gauss = magResp.field_x_gauss;
                  status.magnetY_gauss = magResp.field_y_gauss;
                  status.magnetZ_gauss = magResp.field_z_gauss;
                  status.magnetFieldValid = true;
                  broadcastStatus(status);
                  break;
                }
              }
            }
            if (!respReceived) {
              DEBUG_PRINTF("No magnetic field response from robot 0x%02X (timeout)\n", cmd->targetID);
              GUIStatus status;
              status.targetID = cmd->targetID;
              status.ackReceived = false;
              status.currentX = 0;
              status.currentY = 0;
              status.currentAngle = 0;
              status.timestamp = 0;
              status.batteryVoltage = 0;
              status.magnetFieldValid = false;
              broadcastStatus(status);
            }
          }
        } else {
          DEBUG_PRINTF("Error sending magnetic field request: %d\n", result);
        }
        
      } else {
        // Send position command (requestType == 0)
        PositionCommand posCmd;
        posCmd.targetID = cmd->targetID;
        posCmd.msg_type = MSG_TYPE_POSITION_COMMAND;
        posCmd.timestamp = millis();
        posCmd.target_x_mm = cmd->x;
        posCmd.target_y_mm = cmd->y;
        posCmd.target_a_rad = cmd->angle;
        posCmd.move_duration_ms = cmd->duration;
        
        DEBUG_PRINTF("Sending position command to robot 0x%02X: (%.2f, %.2f) %.2frad %.2fms\n",
                     cmd->targetID, cmd->x, cmd->y, cmd->angle, cmd->duration);
        
        // Clear any pending ACKs for this target
        AckMessage tempAck;
        while (xQueueReceive(ackQueue, &tempAck, 0) == pdPASS) {
          // Clear queue
        }
        
        // Send broadcast
        esp_err_t result = esp_now_send(broadcastAddress, (uint8_t*)&posCmd, sizeof(posCmd));
        
        if (result == ESP_OK) {
          DEBUG_PRINTLN("Position command broadcast sent");
          
          // Wait for ACK with 500ms timeout
          AckMessage ack;
          bool ackReceived = false;
          uint32_t startTime = millis();
          
          while ((millis() - startTime) < 500) {
            if (xQueueReceive(ackQueue, &ack, pdMS_TO_TICKS(10)) == pdPASS) {
              if (ack.responderID == cmd->targetID && ack.msg_type == MSG_TYPE_ACK_MESSAGE) {
                ackReceived = true;
                DEBUG_PRINTF("ACK received from robot 0x%02X\n", cmd->targetID);
                break;
              }
            }
          }
          
          if (!ackReceived) {
            DEBUG_PRINTF("No ACK received from robot 0x%02X (timeout)\n", cmd->targetID);
            // Send timeout notification to all tasks
            GUIStatus status;
            status.targetID = cmd->targetID;
            status.ackReceived = false;
            status.currentX = 0;
            status.currentY = 0;
            status.currentAngle = 0;
            status.timestamp = 0;
            status.batteryVoltage = 0;
            status.magnetFieldValid = false;
            broadcastStatus(status);
          }
        } else {
          DEBUG_PRINTF("Error sending position command: %d\n", result);
        }
        }  // End of GUI command handling
      } else if (msg.commandType == CMD_TYPE_POS_SYNC) {
        // Wait until start of next emag frame
        vTaskDelay(pdMS_TO_TICKS(getTimeToNextFrameUs() / 1000));

        // Broadcast PosSyncCommand to all units
        PosSyncCommand syncCmd;
        syncCmd.targetID = 0xFF;
        syncCmd.msg_type = MSG_TYPE_POS_SYNC_COMMAND;
        syncCmd.timestamp = (uint32_t)esp_timer_get_time();
        syncCmd.timeout_ms = (POS_SYNC_INITIAL_DELAY_MS + (POS_SYNC_BURST_COUNT + 1) * POS_SYNC_BURST_INTERVAL_MS + EMAG_FRAME_LEN_MS);

        DEBUG_PRINTF("Sending PosSyncCommand to all units (timeout=%dms)\n", syncCmd.timeout_ms);

        // Clear any stale ACKs and NACKs from previous transactions
        AckMessage tempAck;
        while (xQueueReceive(ackQueue, &tempAck, 0) == pdPASS) {}
        NackMessage tempNack;
        while (xQueueReceive(nackQueue, &tempNack, 0) == pdPASS) {}

        esp_err_t result = esp_now_send(broadcastAddress, (uint8_t*)&syncCmd, sizeof(syncCmd));
        if (result != ESP_OK) {
          DEBUG_PRINTF("PosSyncCommand send failed: %d\n", result);
          break;
        }

        DEBUG_PRINTLN("PosSyncCommand broadcast sent");

        // Send a burst of sync pulses to combat Wi-Fi latency
        vTaskDelay(pdMS_TO_TICKS(POS_SYNC_INITIAL_DELAY_MS));
        PosSync syncPulse;
        syncPulse.targetID = 0xFF;
        syncPulse.msg_type = MSG_TYPE_POS_SYNC;
        for (int i = 0; i < POS_SYNC_BURST_COUNT; i++) {
          syncPulse.timestamp = (uint32_t)esp_timer_get_time();
          syncPulse.next_frame_us = getTimeToNextFrameUs();
          result = esp_now_send(broadcastAddress, (uint8_t*)&syncPulse, sizeof(syncPulse));
          if (result != ESP_OK) {
            DEBUG_PRINTF("Error sending PosSync pulse %d: %d\n", i, result);
          }
          if (i < POS_SYNC_BURST_COUNT - 1) {
            vTaskDelay(pdMS_TO_TICKS(POS_SYNC_BURST_INTERVAL_MS));
          }
        }

        // Collect ACK/NACK responses for 5 * EMAG_FRAME_LEN_MS
        uint32_t collectStart = millis();
        while ((millis() - collectStart) < (5 * EMAG_FRAME_LEN_MS)) {
          AckMessage ack;
          if (xQueueReceive(ackQueue, &ack, pdMS_TO_TICKS(5)) == pdPASS) {
            if (ack.msg_type == MSG_TYPE_ACK_MESSAGE) {
              DEBUG_PRINTF("Sync ACK from robot 0x%02X\n", ack.responderID);
              GUIStatus syncStatus = {};
              syncStatus.targetID = ack.responderID;
              syncStatus.ackReceived = true;
              syncStatus.batteryVoltage = -1.0f;
              syncStatus.syncStatus = 1;
              broadcastStatus(syncStatus);
            }
          }
          NackMessage nack;
          if (xQueueReceive(nackQueue, &nack, 0) == pdPASS) {
            if (nack.msg_type == MSG_TYPE_NACK_MESSAGE) {
              DEBUG_PRINTF("Sync NACK from robot 0x%02X (err=%d)\n", nack.responderID, nack.err_type);
              GUIStatus syncStatus = {};
              syncStatus.targetID = nack.responderID;
              syncStatus.ackReceived = false;
              syncStatus.batteryVoltage = -1.0f;
              syncStatus.syncStatus = 2;
              broadcastStatus(syncStatus);
            }
          }
        DEBUG_PRINTLN("Sync response collection window closed");
        }
      }  // End of command type check
    }
    
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}
