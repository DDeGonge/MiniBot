#include "motor_test_queue.h"

MotorTestQueue MotorTestQueue_Create(uint16_t max_queue_size) {
  return xQueueCreate(max_queue_size, sizeof(MotorTestRequest));
}

bool MotorTestQueue_IsEmpty(MotorTestQueue queue) {
  if (queue == NULL) {
    return true;
  }
  return uxQueueMessagesWaiting(queue) == 0;
}

bool MotorTestQueue_IsFull(MotorTestQueue queue) {
  if (queue == NULL) {
    return true;
  }
  return uxQueueSpacesAvailable(queue) == 0;
}

uint16_t MotorTestQueue_GetSize(MotorTestQueue queue) {
  if (queue == NULL) {
    return 0;
  }
  return (uint16_t)uxQueueMessagesWaiting(queue);
}

bool MotorTestQueue_Enqueue(MotorTestQueue queue, MotorTestRequest *command) {
  if (queue == NULL || command == NULL) {
    return false;
  }
  return xQueueSend(queue, command, 0) == pdTRUE;
}

bool MotorTestQueue_EnqueueFromISR(MotorTestQueue queue,
                                   MotorTestRequest *command) {
  if (queue == NULL || command == NULL) {
    return false;
  }
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  BaseType_t result =
      xQueueSendFromISR(queue, command, &xHigherPriorityTaskWoken);
  if (xHigherPriorityTaskWoken == pdTRUE) {
    portYIELD_FROM_ISR();
  }
  return result == pdTRUE;
}

bool MotorTestQueue_Dequeue(MotorTestQueue queue, MotorTestRequest *command,
                            uint32_t timeout_ms) {
  if (queue == NULL || command == NULL) {
    return false;
  }
  TickType_t timeout_ticks =
      (timeout_ms == portMAX_DELAY) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
  return xQueueReceive(queue, command, timeout_ticks) == pdTRUE;
}

void MotorTestQueue_Destroy(MotorTestQueue queue) {
  if (queue != NULL) {
    vQueueDelete(queue);
  }
}
