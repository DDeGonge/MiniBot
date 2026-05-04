#include "JoystickTask.h"

#if ENABLE_JOYSTICK_MODE

#include "QueueStructs.h"
#include "config.h"

TaskHandle_t joystickTaskHandle = NULL;

struct JoystickState {
  float lastThrottle;
  float lastSteering;
  uint32_t lastSendTime;
  uint32_t lastChangeTime;
};
 
void initJoystick() {
  pinMode(JOYSTICK_THROTTLE_PIN, INPUT);
  pinMode(JOYSTICK_STEERING_PIN, INPUT);
  DEBUG_PRINTLN("Joystick initialized on pins: T=" + String(JOYSTICK_THROTTLE_PIN) + 
                 ", S=" + String(JOYSTICK_STEERING_PIN));
}

float normalizeInput(uint16_t rawValue, uint16_t center, uint16_t deadzone, int8_t invert) {
  int16_t offset = (int16_t)rawValue - (int16_t)center;


  // If within deadzone, return 0
  if (abs(offset) <= deadzone) {
    return 0.0f;
  }

  float norm = 0.0f;
  if (offset > 0) {
    int16_t maxOffset = (int16_t)JOYSTICK_MAX_TICK - (int16_t)center - deadzone;
    if (maxOffset <= 0) return 1.0f * invert;
    norm = (float)(offset - deadzone) / (float)maxOffset;
    if (norm > 1.0f) norm = 1.0f;
  } else {
    int16_t minOffset = (int16_t)center - (int16_t)JOYSTICK_MIN_TICK - deadzone;
    if (minOffset <= 0) return -1.0f * invert;
    norm = (float)(offset + deadzone) / (float)minOffset;
    if (norm < -1.0f) norm = -1.0f;
  }
  return norm * invert;
}

// Calculate motor velocities for differential drive
void calculateMotorVelocities(float throttleInput, float steeringInput, 
                              int8_t &m0_vel, int8_t &m1_vel) {
  
  // Reduce steering influence as throttle increases
  float steeringInfluence = 1.0f - (abs(throttleInput) * JOYSTICK_STEERING_INFLUENCE_FACTOR);
  steeringInput *= steeringInfluence;
  steeringInput *= JOYSTICK_STEERING_SCALE;
  
  // Differential drive motor control
  float m0_float = throttleInput + steeringInput;
  float m1_float = throttleInput - steeringInput;
  
  // Normalize if either exceeds max
  float maxVal = max(abs(m0_float), abs(m1_float));
  if (maxVal > 1.0f) {
    m0_float /= maxVal;
    m1_float /= maxVal;
  }

  DEBUG_PRINTF("Normalized Inputs -> Throttle: %.2f, Steering: %.2f, Influence: %.2f, M0: %.2f, M1: %.2f\n", 
                throttleInput, steeringInput, steeringInfluence, m0_float, m1_float);
  
  // Scale to motor velocity range
  m0_vel = (int8_t)(m0_float * JOYSTICK_MAX_MOTOR_VELOCITY);
  m1_vel = (int8_t)(m1_float * JOYSTICK_MAX_MOTOR_VELOCITY);
}

bool hasSignalChanged(float throttleNow, float steeringNow, 
                      float lastThrottle, float lastSteering) {
  float changeThreshold = 0.05f; // 5% change threshold

  bool thresholdExceeded = (abs(throttleNow - lastThrottle) > changeThreshold) ||
                           (abs(steeringNow - lastSteering) > changeThreshold);

  bool throttleCrossedZero = (lastThrottle != 0.0f && throttleNow == 0.0f);
  bool steeringCrossedZero = (lastSteering != 0.0f && steeringNow == 0.0f);
  
  return thresholdExceeded || throttleCrossedZero || steeringCrossedZero;
}

// FreeRTOS Joystick task
void joystickTask(void *parameter) {
  JoystickState state;
  state.lastThrottle = 0;
  state.lastSteering = 0;
  state.lastSendTime = millis();
  state.lastChangeTime = millis();
  
  DEBUG_PRINTLN("Joystick Task started");
  
  static uint32_t lastDiagPrint = 0;

  while (true) {
    uint32_t currentTime = millis();
    uint16_t throttleRaw = analogRead(JOYSTICK_THROTTLE_PIN);
    uint16_t steeringRaw = analogRead(JOYSTICK_STEERING_PIN);
    
    float throttleInput = normalizeInput(throttleRaw, JOYSTICK_THROTTLE_CENTER, 
                                          JOYSTICK_DEADZONE, JOYSTICK_THROTTLE_INVERT);
    float steeringInput = normalizeInput(steeringRaw, JOYSTICK_STEERING_CENTER, 
                                          JOYSTICK_DEADZONE, JOYSTICK_STEERING_INVERT);

    // Periodic diagnostic: print raw ADC and normalized values every second
    if (currentTime - lastDiagPrint >= 1000) {
      Serial.printf("[JOY] Raw T:%d S:%d  Norm T:%.2f S:%.2f  timeSinceSend:%ums\n",
                    throttleRaw, steeringRaw, throttleInput, steeringInput,
                    (unsigned)(currentTime - state.lastSendTime));
      lastDiagPrint = currentTime;
    }

    // Determine if we should send a command
    bool shouldSend = false;
    uint32_t timeSinceSend = currentTime - state.lastSendTime;
    
    if (hasSignalChanged(throttleInput, steeringInput, state.lastThrottle, state.lastSteering)) {
      if (timeSinceSend >= JOYSTICK_UPDATE_INTERVAL_MS) {
        shouldSend = true;
        state.lastChangeTime = currentTime;
        state.lastThrottle = throttleInput;
        state.lastSteering = steeringInput;
      }
    } else if (timeSinceSend >= JOYSTICK_WATCHDOG_INTERVAL_MS) {
      shouldSend = true;
    }
    
    if (shouldSend) {
      DEBUG_PRINTF("Joystick Raw[T:%d, S:%d]  ", (int)throttleRaw, (int)steeringRaw);
      if (throttleInput == 0 && steeringInput == 0) {
        // If joystick is centered, send a stop command
        CommandMessage msg;
        msg.commandType = CMD_TYPE_MOT_TEST;
        msg.data.motCmd.targetID = JOYSTICK_TARGET_ID;
        msg.data.motCmd.msg_type = MSG_TYPE_MOT_TEST_COMMAND;
        msg.data.motCmd.enabled = 0;
        msg.data.motCmd.m0_vel = 0;
        msg.data.motCmd.m1_vel = 0;
        
        Serial.printf("[JOY] Queuing STOP command\n");
        if (xQueueSend(commandQueue, &msg, pdMS_TO_TICKS(10)) != pdPASS) {
          Serial.println("[JOY] WARNING: Command queue full, dropping stop command");
        }
      } else {
        int8_t m0_vel, m1_vel;
        calculateMotorVelocities(throttleInput, steeringInput, m0_vel, m1_vel);

        CommandMessage msg;
        msg.commandType = CMD_TYPE_MOT_TEST;
        msg.data.motCmd.targetID = JOYSTICK_TARGET_ID;
        msg.data.motCmd.msg_type = MSG_TYPE_MOT_TEST_COMMAND;
        msg.data.motCmd.enabled = 1;
        msg.data.motCmd.m0_vel = m0_vel;
        msg.data.motCmd.m1_vel = m1_vel;

        Serial.printf("[JOY] Queuing cmd: T:%.2f S:%.2f -> M0:%d M1:%d\n",
                      throttleInput, steeringInput, (int)m0_vel, (int)m1_vel);
        
        if (xQueueSend(commandQueue, &msg, pdMS_TO_TICKS(10)) != pdPASS) {
          Serial.println("[JOY] WARNING: Command queue full, dropping command");
        }
      }
      
      state.lastSendTime = currentTime;
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

#endif // ENABLE_JOYSTICK_MODE
