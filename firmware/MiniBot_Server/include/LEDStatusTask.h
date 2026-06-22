#ifndef LED_STATUS_TASK_H
#define LED_STATUS_TASK_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

// -------------------------------------------------------
// LED event types
// -------------------------------------------------------
typedef enum {
  LED_EVENT_IDLE =
      0,            // Return to default heartbeat (100ms on / 900ms off, white)
  LED_EVENT_ERROR,  // Fast red blink until cleared
  LED_EVENT_WARN,   // Slow amber (white) blink, N times then return to idle
  LED_EVENT_CUSTOM, // Caller-controlled: on_ms / off_ms / colour / repeat count
} LedEventType;

typedef enum {
  LED_COLOR_WHITE = 0,
  LED_COLOR_RED,
} LedColor;

typedef struct {
  LedEventType type;
  LedColor color;
  uint16_t on_ms;  // LED on duration (ms)
  uint16_t off_ms; // LED off duration (ms)
  int16_t repeats; // Number of blink cycles; -1 = indefinite until next event
} LedEvent;

// Task handle
extern TaskHandle_t ledStatusTaskHandle;

// Queue – post a LedEvent from any task to change the LED state
extern QueueHandle_t ledEventQueue;

// Initialize LED pins and create the event queue
void initLEDStatus();

// FreeRTOS LED status task
void ledStatusTask(void *parameter);

// Convenience helpers
void ledPostEvent(LedEventType type, LedColor color = LED_COLOR_WHITE,
                  uint16_t on_ms = 100, uint16_t off_ms = 900,
                  int16_t repeats = -1);

#endif // LED_STATUS_TASK_H
