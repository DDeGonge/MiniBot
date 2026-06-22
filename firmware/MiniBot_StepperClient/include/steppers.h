#ifndef __STEPPERS_H__
#define __STEPPERS_H__

// Default stepper parameters
static constexpr float MAX_ACCEL_RAD_S2 = 10.0f; // wheel accel limit
static constexpr float CONTROL_PERIOD_MS =
    10.0f; // control update period for ramp (10ms)

// Function protos
float wheelRadS2ToStepsS2(float rad_s2);
void StepperTask(void *pvParameters);

#endif // __STEPPERS_H__