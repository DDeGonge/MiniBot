#include "LEDStatusTask.h"
#include "config.h"

#if ENABLE_JOYSTICK_MODE
#define IDLE_LED_COLOR LED_COLOR_RED
#else
#define IDLE_LED_COLOR LED_COLOR_WHITE
#endif

TaskHandle_t ledStatusTaskHandle = NULL;
QueueHandle_t ledEventQueue = NULL;

// Apply a color to the LED (LOW = off, HIGH = on for the matching pin)
static void setLed(LedColor color, bool on) {
  digitalWrite(STATUS_LED_WHITE_PIN,
               (color == LED_COLOR_WHITE && on) ? HIGH : LOW);
  digitalWrite(STATUS_LED_RED_PIN, (color == LED_COLOR_RED && on) ? HIGH : LOW);
}

// Turn all LEDs off
static void ledsOff() {
  digitalWrite(STATUS_LED_WHITE_PIN, LOW);
  digitalWrite(STATUS_LED_RED_PIN, LOW);
}

void initLEDStatus() {
  pinMode(STATUS_LED_WHITE_PIN, OUTPUT);
  pinMode(STATUS_LED_RED_PIN, OUTPUT);
  ledsOff();

  ledEventQueue = xQueueCreate(8, sizeof(LedEvent));
  DEBUG_PRINTLN("LED Status initialized");
}

void ledPostEvent(LedEventType type, LedColor color, uint16_t on_ms,
                  uint16_t off_ms, int16_t repeats) {
  if (ledEventQueue == NULL)
    return;
  LedEvent ev = {type, color, on_ms, off_ms, repeats};
  // Drop oldest if full so callers never block
  if (xQueueSend(ledEventQueue, &ev, 0) != pdPASS) {
    LedEvent discard;
    xQueueReceive(ledEventQueue, &discard, 0);
    xQueueSend(ledEventQueue, &ev, 0);
  }
}

void ledStatusTask(void *parameter) {
  DEBUG_PRINTLN("LED Status Task started");

  // Default idle heartbeat: 100ms on, 900ms off, indefinite
  LedEvent current = {
      .type = LED_EVENT_IDLE,
      .color = IDLE_LED_COLOR,
      .on_ms = 100,
      .off_ms = 900,
      .repeats = -1,
  };

  int16_t remainingRepeats = current.repeats;

  while (1) {
    // --- ON phase ---
    setLed(current.color, true);
    // Poll queue during the on period; if an event arrives, apply it
    // immediately
    LedEvent incoming;
    if (xQueueReceive(ledEventQueue, &incoming, pdMS_TO_TICKS(current.on_ms)) ==
        pdPASS) {
      ledsOff();
      current = incoming;
      remainingRepeats = current.repeats;
      continue; // restart cycle with new event
    }

    // --- OFF phase ---
    ledsOff();
    if (xQueueReceive(ledEventQueue, &incoming,
                      pdMS_TO_TICKS(current.off_ms)) == pdPASS) {
      current = incoming;
      remainingRepeats = current.repeats;
      continue;
    }

    // --- Count down finite repeats ---
    if (remainingRepeats > 0) {
      remainingRepeats--;
      if (remainingRepeats == 0) {
        // Finished: return to idle
        current = {
            .type = LED_EVENT_IDLE,
            .color = IDLE_LED_COLOR,
            .on_ms = 100,
            .off_ms = 900,
            .repeats = -1,
        };
        remainingRepeats = -1;
      }
    }
  }
}
