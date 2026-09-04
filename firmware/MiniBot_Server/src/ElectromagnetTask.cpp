#include "ElectromagnetTask.h"
#include <esp_timer.h>
#include "MCP23S17.h"

// Task handle
TaskHandle_t emagTaskHandle = NULL;

// Control flags
volatile bool emagEnabled = false;
volatile int64_t nextFrameStartUs = 0;
static MCP ioExpander(0, 5);
static Electromagnet EMAGNETS[EMAG_COUNT];
static unsigned int expanderOutputCache = 0;

Electromagnet::Electromagnet() {}
Electromagnet::Electromagnet(bool on_io_expander, uint8_t en_pin, uint8_t dir_pin)
    : en_pin(en_pin), dir_pin(dir_pin), on_io_expander(on_io_expander) {}

void Electromagnet::init(bool on_io_expander, uint8_t en_pin, uint8_t dir_pin) {
  this->on_io_expander = on_io_expander;
  this->en_pin = en_pin;
  this->dir_pin = dir_pin;
  this->enabled = false;
  this->forward = true;
}

void Electromagnet::set(bool enabled, bool forward) {
  setDirection(forward);
  enable(enabled);
}

void Electromagnet::enable(bool enabled_) {
  enabled = enabled_;
  if (on_io_expander) {
    // update cached output bit and write full word once
    if (enabled)
      expanderOutputCache |= (1u << en_pin);
    else
      expanderOutputCache &= ~(1u << en_pin);
    ioExpander.digitalWrite(expanderOutputCache);
  } else {
    digitalWrite(en_pin, enabled ? HIGH : LOW);
  }
}

void Electromagnet::setDirection(bool forward_) {
  forward = forward_;
  // Direction pin is shared and assumed to be a GPIO
  if (flip_emag_direction) {
    digitalWrite(dir_pin, forward ? LOW : HIGH);
  } else {
    digitalWrite(dir_pin, forward ? HIGH : LOW);
  }
}

bool Electromagnet::isEnabled() const { return enabled; }

bool Electromagnet::isForward() const { return forward; }

// Initialize electromagnets
void initElectromagnets() {
  if (EMAG_COUNT * (EMAG_FWD_ON_TIME_MS + EMAG_REV_ON_TIME_MS) >
      EMAG_FRAME_LEN_MS) {
    Serial.println("WARNING: Electromagnet on-time exceeds frame length! Adjust "
                  "timing parameters.");
  }

  // Initialize IO expander and shared direction pin
  ioExpander.begin(); // uses the default MOSI MISO SCK pins for esp32
  pinMode(EMAG_DIR_PIN, OUTPUT);
  digitalWrite(EMAG_DIR_PIN, LOW);

  // Configure each electromagnet enable pin (either GPIO or expander)
  expanderOutputCache = 0;
  for (int i = 0; i < EMAG_COUNT; i++) {
    if (EMAG_EN_ON_EXPANDER[i]) {
      // MCP library pin numbers are 1..16; our EMAG_EN_PINS are 0-based.
      ioExpander.pinMode(EMAG_EN_PINS[i] + 1, OUTPUT); // Configure as output (1-based)
      expanderOutputCache &= ~(1u << EMAG_EN_PINS[i]);
    } else {
      pinMode(EMAG_EN_PINS[i], OUTPUT);
      digitalWrite(EMAG_EN_PINS[i], LOW);
    }
    EMAGNETS[i].init(EMAG_EN_ON_EXPANDER[i], EMAG_EN_PINS[i], EMAG_DIR_PIN);
  }
  // ensure expander outputs are cleared on the chip
  ioExpander.digitalWrite(expanderOutputCache);

  Serial.println("Electromagnets initialized");
}

bool setElectromagnet(uint8_t emag_i, bool enabled, bool forward) {
  if (emag_i >= EMAG_COUNT) {
    Serial.printf("Invalid electromagnet index: %d\n", emag_i);
    return false;
  }
  EMAGNETS[emag_i].set(enabled, forward);
  return true;
}

// Set all electromagnets: disabled=[0,0], forward=[1,0], reverse=[0,1]
void setAllElectromagnets(bool enabled, bool forward) {
  // Set shared direction first
  if (flip_emag_direction) {
    digitalWrite(EMAG_DIR_PIN, forward ? LOW : HIGH);
  } else {
    digitalWrite(EMAG_DIR_PIN, forward ? HIGH : LOW);
  }

  // Build expander word and set GPIOs
  unsigned int expanderWord = 0;
  for (int i = 0; i < EMAG_COUNT; i++) {
    if (EMAG_EN_ON_EXPANDER[i]) {
      if (enabled)
        expanderWord |= (1u << EMAG_EN_PINS[i]);
    } else {
      digitalWrite(EMAG_EN_PINS[i], enabled ? HIGH : LOW);
    }
  }
  ioExpander.digitalWrite(expanderWord);
  expanderOutputCache = expanderWord;
}

// Enable/disable electromagnet cycling
void setElectromagnetEnabled(bool enabled) {
  emagEnabled = enabled;
  if (!enabled) {
    setAllElectromagnets(false);
  }
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
  Serial.println("Electromagnet Task started");

  const int64_t frameLenUs = (int64_t)EMAG_FRAME_LEN_MS * 1000LL;
  nextFrameStartUs = esp_timer_get_time();

  while (1) {
    nextFrameStartUs += frameLenUs;
    if (!waitUntilUs(nextFrameStartUs)) {
      Serial.println("Frame skipped: overrun at frame start");
      continue;
    }
    if (!emagEnabled) {
      continue;
    }

    // --- Frame start ---
    const int64_t fwdOnUs = (int64_t)EMAG_FWD_ON_TIME_MS * 1000LL;
    const int64_t revOnUs = (int64_t)EMAG_REV_ON_TIME_MS * 1000LL;
    int64_t interFrameTimeUs = nextFrameStartUs;

    // DEBUG LOOP
    // setAllElectromagnets(true, true);
    // vTaskDelay(pdMS_TO_TICKS((uint32_t)30));

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
