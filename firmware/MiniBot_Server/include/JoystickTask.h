#ifndef JOYSTICK_TASK_H
#define JOYSTICK_TASK_H

#include "ESPNowMessages.h"
#include "config.h"
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#if ENABLE_JOYSTICK_MODE

// Task handle
extern TaskHandle_t joystickTaskHandle;

// Initialize Joystick inputs
void initJoystick();

// FreeRTOS Joystick task
void joystickTask(void *parameter);

#endif // ENABLE_JOYSTICK_MODE

#endif // JOYSTICK_TASK_H
