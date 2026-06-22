#ifndef __MOTION_QUEUE_H__
#define __MOTION_QUEUE_H__

#include "messages_ipc.h"
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <stdbool.h>
#include <stdint.h>

/**
 * Motion queue handle - wrapper around FreeRTOS queue
 */
typedef QueueHandle_t MotionQueue;

/**
 * Initialize a new motion queue
 * @param max_queue_size Maximum number of commands in the queue
 * @return Queue handle, or NULL on failure
 */
MotionQueue MotionQueue_Create(uint16_t max_queue_size);

/**
 * Check if queue is empty
 * @param queue Queue handle
 * @return true if empty, false otherwise
 */
bool MotionQueue_IsEmpty(MotionQueue queue);

/**
 * Check if queue is full
 * @param queue Queue handle
 * @return true if full, false otherwise
 */
bool MotionQueue_IsFull(MotionQueue queue);

/**
 * Get current size of queue
 * @param queue Queue handle
 * @return Current number of commands in queue
 */
uint16_t MotionQueue_GetSize(MotionQueue queue);

/**
 * Add a command to the queue (non-blocking)
 * @param queue Queue handle
 * @param command Pointer to MotionCommand to enqueue
 * @return true on success, false if queue is full
 */
bool MotionQueue_Enqueue(MotionQueue queue, MotionCommand *command);

/**
 * Add a command to the queue from ISR context
 * @param queue Queue handle
 * @param command Pointer to MotionCommand to enqueue
 * @return true on success, false if queue is full
 */
bool MotionQueue_EnqueueFromISR(MotionQueue queue, MotionCommand *command);

/**
 * Remove a command from the queue (blocking with timeout)
 * @param queue Queue handle
 * @param command Pointer to MotionCommand to populate with dequeued data
 * @param timeout_ms Maximum time to wait in milliseconds (0 for non-blocking)
 * @return true on success, false if queue is empty or timeout
 */
bool MotionQueue_Dequeue(MotionQueue queue, MotionCommand *command,
                         uint32_t timeout_ms);

/**
 * Destroy queue and free all resources
 * @param queue Queue handle
 */
void MotionQueue_Destroy(MotionQueue queue);

#endif // __MOTION_QUEUE_H__
