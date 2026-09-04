#ifndef ELECTROMAGNET_TASK_H
#define ELECTROMAGNET_TASK_H

#include "config.h"
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Task handle
extern TaskHandle_t emagTaskHandle;

// Control flags
extern volatile bool emagEnabled;
extern volatile bool syncPulseRequested;
extern volatile int64_t nextFrameStartUs;

// Initialize electromagnets
void initElectromagnets();

// FreeRTOS electromagnet task
void electromagnetTask(void *parameter);

// Control single electromagnet
bool setElectromagnet(uint8_t emag_i, bool enabled, bool forward = true);

// Control all electromagnets
void setAllElectromagnets(bool enabled, bool forward = true);

// Enable/disable electromagnet position cycle
void setElectromagnetEnabled(bool enabled);

// Get current state
bool getElectromagnetEnabled();

// Returns microseconds until the start of the next emag frame
uint32_t getTimeToNextFrameUs();

class Electromagnet {
    public:
        Electromagnet();
        Electromagnet(bool on_io_expander, uint8_t en_pin, uint8_t dir_pin = EMAG_DIR_PIN);
        void init(bool on_io_expander, uint8_t en_pin, uint8_t dir_pin = EMAG_DIR_PIN);
        void set(bool enabled, bool forward);
        void enable(bool enabled);
        void setDirection(bool forward);
        bool isEnabled() const;
        bool isForward() const;

    private:
        uint8_t en_pin = 0;
        uint8_t dir_pin = EMAG_DIR_PIN;
        bool on_io_expander = false;
        bool enabled = false;
        bool forward = true;
};

#endif // ELECTROMAGNET_TASK_H
