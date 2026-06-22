#include "ElectromagnetTask.h"
#include <esp_timer.h>

// Task handle
TaskHandle_t emagTaskHandle = NULL;

// Control flags
volatile bool emagEnabled = false;
volatile int64_t nextFrameStartUs = 0;

// Initialize electromagnets
void initElectromagnets() {
  if (EMAG_COUNT * (EMAG_FWD_ON_TIME_MS + EMAG_REV_ON_TIME_MS) >
      EMAG_FRAME_LEN_MS) {
    DEBUG_PRINTLN("WARNING: Electromagnet on-time exceeds frame length! Adjust "
                  "timing parameters.");
  }

  // Configure all electromagnet pins as outputs, start disabled
  for (int i = 0; i < EMAG_COUNT; i++) {
    pinMode(EMAG_PINS_A[i], OUTPUT);
    pinMode(EMAG_PINS_B[i], OUTPUT);
    digitalWrite(EMAG_PINS_A[i], LOW);
    digitalWrite(EMAG_PINS_B[i], LOW);
  }

  DEBUG_PRINTLN("Electromagnets initialized");
}

bool setElectromagnet(uint8_t emag_i, bool enabled, bool forward) {
  if (emag_i >= EMAG_COUNT) {
    DEBUG_PRINTF("Invalid electromagnet index: %d\n", emag_i);
    return false;
  }

  if (!enabled) {
    digitalWrite(EMAG_PINS_A[emag_i], LOW);
    digitalWrite(EMAG_PINS_B[emag_i], LOW);
  } else if (forward) {
    digitalWrite(EMAG_PINS_A[emag_i], HIGH);
    digitalWrite(EMAG_PINS_B[emag_i], LOW);
  } else {
    digitalWrite(EMAG_PINS_A[emag_i], LOW);
    digitalWrite(EMAG_PINS_B[emag_i], HIGH);
  }
  return true;
}

// Set all electromagnets: disabled=[0,0], forward=[1,0], reverse=[0,1]
void setAllElectromagnets(bool enabled, bool forward) {
  for (int i = 0; i < EMAG_COUNT; i++) {
    setElectromagnet(i, enabled, forward);
  }
}

// Enable/disable electromagnet cycling
void setElectromagnetEnabled(bool enabled) {
  emagEnabled = enabled;
  if (!enabled) {
    setAllElectromagnets(false);
  }
  DEBUG_PRINTF("Electromagnet cycling %s\n", enabled ? "ENABLED" : "DISABLED");
}

// Get current state
bool getElectromagnetEnabled() { return emagEnabled; }

// Returns microseconds until the start of the next emag frame
uint32_t getTimeToNextFrameUs() {
  const int64_t frameLenUs = (int64_t)EMAG_FRAME_LEN_MS * 1000LL;
  int64_t timeToNext = nextFrameStartUs - esp_timer_get_time();
  if (timeToNext <= 0)
    timeToNext += frameLenUs;
  return (uint32_t)timeToNext;
}

// Wait until an absolute esp_timer_get_time() target (µs).
// Uses vTaskDelay to yield when more than 2ms remain, then busy-waits the tail.
// Returns false if vTaskDelay ran long and targetUs was already passed on
// wakeup.
static bool waitUntilUs(int64_t targetUs) {
  int64_t sleepMs = (targetUs - esp_timer_get_time()) / 1000LL - 2;
  if (sleepMs > 0) {
    vTaskDelay(pdMS_TO_TICKS((uint32_t)sleepMs));
    if (esp_timer_get_time() >= targetUs) {
      return false;
    }
  }
  while (esp_timer_get_time() < targetUs) {
  }
  return true;
}

// FreeRTOS electromagnet task
void electromagnetTask(void *parameter) {
  DEBUG_PRINTLN("Electromagnet Task started");

  const int64_t frameLenUs = (int64_t)EMAG_FRAME_LEN_MS * 1000LL;
  nextFrameStartUs = esp_timer_get_time();

  while (1) {
    nextFrameStartUs += frameLenUs;
    if (!waitUntilUs(nextFrameStartUs)) {
      DEBUG_PRINTLN("Frame skipped: overrun at frame start");
      continue;
    }
    if (!emagEnabled) {
      continue;
    }

    // --- Frame start ---
    const int64_t fwdOnUs = (int64_t)EMAG_FWD_ON_TIME_MS * 1000LL;
    const int64_t revOnUs = (int64_t)EMAG_REV_ON_TIME_MS * 1000LL;
    int64_t interFrameTimeUs = nextFrameStartUs;

    for (int i = 0; i < EMAG_COUNT && emagEnabled; i++) {
      // Forward ON
      setElectromagnet(i, true, true);
      interFrameTimeUs += fwdOnUs;
      if (!waitUntilUs(interFrameTimeUs)) {
        DEBUG_PRINTLN("WARNING: Timing overrun before reverse ON");
      }

      // Reverse ON
      setElectromagnet(i, true, false);
      interFrameTimeUs += revOnUs;
      if (!waitUntilUs(interFrameTimeUs)) {
        DEBUG_PRINTLN("WARNING: Timing overrun before OFF");
      }

      // OFF
      setElectromagnet(i, false);
    }

    // Ensure all emags are off at the end of the frame
    setAllElectromagnets(false);
  }
}
