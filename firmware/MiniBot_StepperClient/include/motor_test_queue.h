#ifndef __MOTOR_TEST_QUEUE_H__
#define __MOTOR_TEST_QUEUE_H__

#include "messages_ipc.h"
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <stdbool.h>
#include <stdint.h>

/**
 * Motor test queue handle - wrapper around FreeRTOS queue
 */
typedef QueueHandle_t MotorTestQueue;

/**
 * Initialize a new motor test queue
 * @param max_queue_size Maximum number of commands in the queue
 * @return Queue handle, or NULL on failure
 */
MotorTestQueue MotorTestQueue_Create(uint16_t max_queue_size);

/**
 * Check if queue is empty
 * @param queue Queue handle
 * @return true if empty, false otherwise
 */
bool MotorTestQueue_IsEmpty(MotorTestQueue queue);

/**
 * Check if queue is full
 * @param queue Queue handle
 * @return true if full, false otherwise
 */
bool MotorTestQueue_IsFull(MotorTestQueue queue);

/**
 * Get current size of queue
 * @param queue Queue handle
 * @return Current number of commands in queue
 */
uint16_t MotorTestQueue_GetSize(MotorTestQueue queue);

/**
 * Add a command to the queue (non-blocking)
 * @param queue Queue handle
 * @param command Pointer to MotorTestRequest to enqueue
 * @return true on success, false if queue is full
 */
bool MotorTestQueue_Enqueue(MotorTestQueue queue, MotorTestRequest *command);

/**
 * Add a command to the queue from ISR context
 * @param queue Queue handle
 * @param command Pointer to MotorTestRequest to enqueue
 * @return true on success, false if queue is full
 */
bool MotorTestQueue_EnqueueFromISR(MotorTestQueue queue,
                                   MotorTestRequest *command);

/**
 * Remove a command from the queue (blocking with timeout)
 * @param queue Queue handle
 * @param command Pointer to MotorTestRequest to populate with dequeued data
 * @param timeout_ms Maximum time to wait in milliseconds (0 for non-blocking)
 * @return true on success, false if queue is empty or timeout
 */
bool MotorTestQueue_Dequeue(MotorTestQueue queue, MotorTestRequest *command,
                            uint32_t timeout_ms);

/**
 * Destroy queue and free all resources
 * @param queue Queue handle
 */
void MotorTestQueue_Destroy(MotorTestQueue queue);

#endif // __MOTOR_TEST_QUEUE_H__
