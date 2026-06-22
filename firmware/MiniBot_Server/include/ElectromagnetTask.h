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

// Request a one-shot 3ms sync pulse at the start of the next emag frame
void triggerSyncPulse();

// Returns microseconds until the start of the next emag frame
uint32_t getTimeToNextFrameUs();

#endif // ELECTROMAGNET_TASK_H
