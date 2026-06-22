#include "motion_queue.h"

MotionQueue MotionQueue_Create(uint16_t max_queue_size) {
  return xQueueCreate(max_queue_size, sizeof(MotionCommand));
}

bool MotionQueue_IsEmpty(MotionQueue queue) {
  if (queue == NULL) {
    return true;
  }
  return uxQueueMessagesWaiting(queue) == 0;
}

bool MotionQueue_IsFull(MotionQueue queue) {
  if (queue == NULL) {
    return true;
  }
  return uxQueueSpacesAvailable(queue) == 0;
}

uint16_t MotionQueue_GetSize(MotionQueue queue) {
  if (queue == NULL) {
    return 0;
  }
  return (uint16_t)uxQueueMessagesWaiting(queue);
}

bool MotionQueue_Enqueue(MotionQueue queue, MotionCommand *command) {
  if (queue == NULL || command == NULL) {
    return false;
  }
  return xQueueSend(queue, command, 0) == pdTRUE;
}

bool MotionQueue_EnqueueFromISR(MotionQueue queue, MotionCommand *command) {
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

bool MotionQueue_Dequeue(MotionQueue queue, MotionCommand *command,
                         uint32_t timeout_ms) {
  if (queue == NULL || command == NULL) {
    return false;
  }
  TickType_t timeout_ticks =
      (timeout_ms == portMAX_DELAY) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
  return xQueueReceive(queue, command, timeout_ticks) == pdTRUE;
}

void MotionQueue_Destroy(MotionQueue queue) {
  if (queue != NULL) {
    vQueueDelete(queue);
  }
}
