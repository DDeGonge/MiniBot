#ifndef __POSITION_ESTIMATOR_H__
#define __POSITION_ESTIMATOR_H__

#include "messages_ipc.h"
#include "robot.h"
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

struct EmagReading {
  float x[MAX_SAMPLES_PER_EMAG];
  float y[MAX_SAMPLES_PER_EMAG];
  float z[MAX_SAMPLES_PER_EMAG];
  bool is_forward[MAX_SAMPLES_PER_EMAG];
  int64_t timestamp_us[MAX_SAMPLES_PER_EMAG];
  uint16_t count;
};

struct EmagFrameData {
  EmagReading readings[EMAG_COUNT];
  float bg_x;
  float bg_y;
  float bg_z;
};

struct ProcessedEmagData {
  bool use_reading = false;
  float magnitude_G = 0.0f;
  float azimuth_angle_rad = 0.0f;
  float elevation_angle_rad = 0.0f;
};

struct CalculatedPosition {
  uint8_t emag_index_0 = 0xFF;
  uint8_t emag_index_1 = 0xFF;
  float pos_x_mm = 0.0f;
  float pos_y_mm = 0.0f;
  float ang_rad = 0.0f;
  float confidence = 0.0f;
};

enum PositionEstState { STATE_MEASURING, STATE_IDLE, STATE_SYNC_LOST };

/**
 * Position Estimator — Sensor Task
 *
 * Priority: HIGH (3)
 * Core: 1
 * Responsible for:
 * - Communicating with the MMC5633 magnetometer at 2 kHz
 * - Detecting the frame start signal (3 short pulses)
 * - Using timing after the start signal to associate each sample
 *   with the correct electromagnet slot
 * - Posting a complete EmagFrameData to the internal queue after
 *   all 6 electromagnet slots have been sampled
 *
 * @param pvParameters Unused (pass NULL or Robot*)
 */
void PositionEstimator_SensorTask(void *pvParameters);

/**
 * Position Estimator — Calculation Task
 *
 * Priority: MEDIUM (2)
 * Core: 1
 * Responsible for:
 * - Blocking on the internal emag frame queue
 * - Computing robot position
 * - Updating the robot's true pose via robot->setTruePose()
 *
 * @param pvParameters Pointer to Robot instance
 */
void PositionEstimator_CalcTask(void *pvParameters);

/**
 * Initialize the position estimator
 * Should be called before starting the task
 *
 * @return true on success, false on failure
 */
bool PositionEstimator_Init(void);

/**
 * Get the latest raw magnetometer field readings
 * Returns the averaged magnetic field values from the magnetometer
 *
 * @param x Pointer to store field_x value in Gauss
 * @param y Pointer to store field_y value in Gauss
 * @param z Pointer to store field_z value in Gauss
 * @return true on success, false if magnetometer not initialized
 */
bool PositionEstimator_GetLatestMagneticField(float *x, float *y, float *z);

/**
 * Set the sync reference time externally (e.g. from an ESP-NOW PosSync
 * message). Sets sync_pulse_time_us and next_frame_time_us to sync_time_us,
 * marks synced, and transitions the state machine to STATE_IDLE.
 *
 * @param sync_time_us  Absolute time (esp_timer_get_time units) of the next
 * frame start
 */
void PositionEstimator_SetSyncTime(int64_t sync_time_us);

#endif // __POSITION_ESTIMATOR_H__
