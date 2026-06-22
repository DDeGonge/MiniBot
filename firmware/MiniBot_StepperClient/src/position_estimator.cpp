#include "position_estimator.h"
#include "mmc5633.h"
#include "utils.h"
#undef LOG_LOCAL_LEVEL
#define LOG_LOCAL_LEVEL LOG_LEVEL_POS_EST
#include "config.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <Arduino.h>
#include <Wire.h>

static const char *TAG = "POS_EST";

static MMC5633NJL mag;
static RollingAverage<10> avgX;
static RollingAverage<10> avgY;
static RollingAverage<10> avgZ;
static bool mag_initialized = false;

static EmagFrameData current_frame;
static PositionEstState current_state = STATE_IDLE;
static int64_t frame_start_time_us = 0;

static QueueHandle_t emag_frame_queue = NULL;

static int64_t sync_pulse_time_us = 0;
static int64_t next_frame_time_us = 0;
static bool synced = false;

static TaskHandle_t s_sensor_task_handle = NULL;
static esp_timer_handle_t s_sample_timer = NULL;

static const float emag_positions[][2] = EMAG_POSITIONS_MM;

bool PositionEstimator_Init(void) {
  emag_frame_queue = xQueueCreate(1, sizeof(EmagFrameData));
  if (emag_frame_queue == NULL) {
    ESP_LOGE(TAG, "Failed to create emag frame queue");
    return false;
  }

  if (!mag.begin(SDA_PIN, SCL_PIN)) {
    ESP_LOGE(TAG, "MMC5633NJL init failed");
    return false;
  }

  ESP_LOGD(TAG, "MMC5633NJL magnetometer ready");

  if (!mag.runSelfTest()) {
    ESP_LOGW(TAG, "Magnetometer self-test failed");
  } else {
    ESP_LOGD(TAG, "Magnetometer self-test passed");
  }

  mag_initialized = true;
  return true;
}

bool PositionEstimator_GetLatestMagneticField(float *x, float *y, float *z) {
  if (!mag_initialized || x == NULL || y == NULL || z == NULL) {
    return false;
  }

  *x = avgX.avg();
  *y = avgY.avg();
  *z = avgZ.avg();
  return true;
}

static void trimAndAverageSamples(const EmagReading *reading, float *avg_x,
                                  float *avg_y, float *avg_z) {
  uint16_t trim_samples = (EMAG_TRIM_MS * 1000) / EMAG_MIN_SAMPLE_PERIOD_US;

  if (reading->count < (trim_samples * 2)) {
    *avg_x = 0.0f;
    *avg_y = 0.0f;
    *avg_z = 0.0f;
    return;
  }

  float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;
  uint16_t valid_samples = 0;

  for (uint16_t i = trim_samples; i < (reading->count - trim_samples); i++) {
    sum_x += reading->x[i];
    sum_y += reading->y[i];
    sum_z += reading->z[i];
    valid_samples++;
  }

  if (valid_samples > 0) {
    *avg_x = sum_x / valid_samples;
    *avg_y = sum_y / valid_samples;
    *avg_z = sum_z / valid_samples;
  } else {
    *avg_x = 0.0f;
    *avg_y = 0.0f;
    *avg_z = 0.0f;
  }
}

static bool noiseCorrectedAngle(const float fwd_x, const float fwd_y,
                                const float rev_x, const float rev_y,
                                float *angle_rad) {
  float fwd_azimuth = atan2f(fwd_y, fwd_x);
  float rev_azimuth = atan2f(rev_y, rev_x) + (float)M_PI;
  if (rev_azimuth > (float)M_PI)
    rev_azimuth -= 2.0f * (float)M_PI;

  // calculate the average angle accounting for wraparound
  float angle_delta =
      min(fabsf(fwd_azimuth - rev_azimuth),
          2.0f * (float)M_PI - fabsf(fwd_azimuth - rev_azimuth));
  if (angle_delta > EMAG_MAX_ANGLE_DELTA_RAD) {
    return false;
  }

  if (fabsf(fwd_azimuth - rev_azimuth) > (float)M_PI) {
    // Angles straddle the ±π boundary — shift the sum up by 2π before
    // averaging, then normalize
    *angle_rad = (fwd_azimuth + rev_azimuth + 2.0f * (float)M_PI) / 2.0f;
    if (*angle_rad > (float)M_PI)
      *angle_rad -= 2.0f * (float)M_PI;
  } else {
    *angle_rad = (fwd_azimuth + rev_azimuth) / 2.0f;
  }

  return true;
}

static void sensorToRobotFrame(float sensor_x, float sensor_y,
                               float sensor_theta, float *robot_x,
                               float *robot_y, float *robot_theta) {
  // Sensor is mounted upside-down: Y-axis is mirrored, which negates the
  // heading
  float theta = sensor_theta;

  // Translate from sensor position to robot center using body-frame offset
  float cos_t = cosf(theta);
  float sin_t = sinf(theta);
  *robot_x = sensor_x - SENSOR_OFFSET_X_MM * cos_t + SENSOR_OFFSET_Y_MM * sin_t;
  *robot_y = sensor_y - SENSOR_OFFSET_X_MM * sin_t - SENSOR_OFFSET_Y_MM * cos_t;
  *robot_theta = theta;
}

static uint8_t processEmagReadings(const EmagFrameData *frame,
                                   ProcessedEmagData *processed_data) {
  uint8_t valid_count = 0;
  for (uint8_t i = 0; i < EMAG_COUNT; i++) {
    processed_data[i].use_reading = false;
  }

  ESP_LOGD(TAG, "Background field: X=%.4f  Y=%.4f  Z=%.4f Gauss", frame->bg_x,
           frame->bg_y, frame->bg_z);

  for (uint8_t i = 0; i < EMAG_COUNT; i++) {
    const EmagReading *r = &frame->readings[i];
    ESP_LOGD(TAG, "--- EMAG[%u]  samples=%u ---", i, r->count);

    float sum_fwd_x = 0.0f, sum_fwd_y = 0.0f, sum_fwd_z = 0.0f;
    uint16_t fwd_count = 0;
    float sum_rev_x = 0.0f, sum_rev_y = 0.0f, sum_rev_z = 0.0f;
    uint16_t rev_count = 0;

    static constexpr uint8_t CSV_SAMPLE_SLOTS = 6;
    float fwd_az[CSV_SAMPLE_SLOTS], fwd_mag[CSV_SAMPLE_SLOTS];
    float rev_az[CSV_SAMPLE_SLOTS], rev_mag[CSV_SAMPLE_SLOTS];
    uint8_t fwd_csv = 0, rev_csv = 0;

    for (uint16_t s = 0; s < r->count; s++) {
      float sx = r->x[s] - frame->bg_x;
      float sy = r->y[s] - frame->bg_y;
      float sz = r->z[s] - frame->bg_z;
      ESP_LOGD(TAG, "  sample %u:\tts: %lld\tX=%7.4f\tY=%7.4f\tZ=%7.4f\t%s", s,
               r->timestamp_us[s], sx, sy, sz,
               r->is_forward[s] ? "FWD" : "REV");
      float sample_az = atan2f(sy, sx);
      float sample_mag = sqrtf(sx * sx + sy * sy + sz * sz);
      if (r->is_forward[s]) {
        sum_fwd_x += sx;
        sum_fwd_y += sy;
        sum_fwd_z += sz;
        fwd_count++;
        if (fwd_csv < CSV_SAMPLE_SLOTS) {
          fwd_az[fwd_csv] = sample_az;
          fwd_mag[fwd_csv] = sample_mag;
          fwd_csv++;
        }
      } else {
        sum_rev_x += sx;
        sum_rev_y += sy;
        sum_rev_z += sz;
        rev_count++;
        if (rev_csv < CSV_SAMPLE_SLOTS) {
          rev_az[rev_csv] = sample_az;
          rev_mag[rev_csv] = sample_mag;
          rev_csv++;
        }
      }
    }

    // Serial.printf("%u", i);
    // for (uint8_t j = 0; j < CSV_SAMPLE_SLOTS; j++) {
    //     if (j < fwd_csv) Serial.printf(",%.4f,%.4f", fwd_az[j], fwd_mag[j]);
    //     else Serial.print(",,");
    // }
    // for (uint8_t j = 0; j < CSV_SAMPLE_SLOTS; j++) {
    //     if (j < rev_csv) Serial.printf(",%.4f,%.4f", rev_az[j], rev_mag[j]);
    //     else Serial.print(",,");
    // }
    // Serial.println();

    if (fwd_count > 0 && rev_count > 0) {
      float inv_fwd = 1.0f / fwd_count;
      float avg_fwd_x = sum_fwd_x * inv_fwd;
      float avg_fwd_y = sum_fwd_y * inv_fwd;
      float avg_fwd_z = sum_fwd_z * inv_fwd;

      float inv_rev = 1.0f / rev_count;
      float avg_rev_x = sum_rev_x * inv_rev;
      float avg_rev_y = sum_rev_y * inv_rev;
      float avg_rev_z = sum_rev_z * inv_rev;

      float diff_x = avg_fwd_x - avg_rev_x;
      float diff_y = avg_fwd_y - avg_rev_y;
      float diff_z = avg_fwd_z - avg_rev_z;
      processed_data[i].magnitude_G =
          sqrtf(diff_x * diff_x + diff_y * diff_y + diff_z * diff_z);

      ESP_LOGD(TAG, "  fwd avg: X=%7.2f  Y=%7.2f  Z=%7.2f  (n=%u)", avg_fwd_x,
               avg_fwd_y, avg_fwd_z, fwd_count);
      ESP_LOGD(TAG, "  rev avg: X=%7.2f  Y=%7.2f  Z=%7.2f  (n=%u)", avg_rev_x,
               avg_rev_y, avg_rev_z, rev_count);

      if (processed_data[i].magnitude_G >= EMAG_MIN_SIGNAL_GAUSS) {
        ESP_LOGD(TAG, "  signal strength: %.4f Gauss [VALID]",
                 processed_data[i].magnitude_G);
        bool valid = true;

        float azimuth_rad;
        if (noiseCorrectedAngle(avg_fwd_x, avg_fwd_y, avg_rev_x, avg_rev_y,
                                &azimuth_rad)) {
          processed_data[i].azimuth_angle_rad = azimuth_rad;
        } else {
          ESP_LOGD(TAG,
                   "Large azimuth angle difference, skipping calculation for "
                   "emag[%u]",
                   i);
          valid = false;
        }

        // TODO integrate elevation angle calculations for improved accuracy
        // float elevation_rad;
        // if (noiseCorrectedAngle(avg_fwd_z, sqrtf(avg_fwd_x * avg_fwd_x +
        // avg_fwd_y * avg_fwd_y), avg_rev_z, sqrtf(avg_rev_x * avg_rev_x +
        // avg_rev_y * avg_rev_y), &elevation_rad)) {
        //     processed_data[i].elevation_angle_rad = elevation_rad;
        // }
        // else {
        //     ESP_LOGD(TAG, "Large elevation angle difference, skipping
        //     calculation for emag[%u]", i); valid = false;
        // }

        if (valid) {
          processed_data[i].use_reading = true;
          valid_count++;
        }
      } else {
        ESP_LOGD(
            TAG,
            "  signal strength: %.4f Gauss [INVALID - below signal threshold]",
            processed_data[i].magnitude_G);
      }
    } else {
      ESP_LOGD(TAG, "  emag[%u]: insufficient fwd/rev samples - skipped", i);
    }
  }
  ESP_LOGD(TAG, "Total valid emags: %u", valid_count);
  return valid_count;
}

static bool populateEmagIndices(CalculatedPosition *calculated_positions,
                                ProcessedEmagData *processed_data,
                                uint8_t valid_emag_count) {
  for (uint8_t i = 0; i < 3; i++) {
    calculated_positions[i].emag_index_0 = 0xFF;
    calculated_positions[i].emag_index_1 = 0xFF;
    calculated_positions[i].confidence = 0.0f;
  }

  ESP_LOGD(TAG, "Valid emag count: %u", valid_emag_count);
  uint8_t indices[3] = {0xFF, 0xFF, 0xFF};
  switch (valid_emag_count) {
  case 0:
  case 1:
    return false;
  case 2:
    for (uint8_t i = 0, j = 0; i < EMAG_COUNT && j < 2; i++) {
      if (processed_data[i].use_reading) {
        indices[j++] = i;
      }
    }
    calculated_positions[0].emag_index_0 = indices[0];
    calculated_positions[0].emag_index_1 = indices[1];
    return true;
  case 3:
    for (uint8_t i = 0, j = 0; i < EMAG_COUNT && j < 3; i++) {
      if (processed_data[i].use_reading) {
        indices[j++] = i;
      }
    }
    break;
  default: {
    // 4 or more valid — select the 3 strongest by iterative max search
    for (uint8_t pick = 0; pick < 3; pick++) {
      float best_mag = -1.0f;
      for (uint8_t i = 0; i < EMAG_COUNT; i++) {
        if (!processed_data[i].use_reading)
          continue;
        bool already_picked = false;
        for (uint8_t p = 0; p < pick; p++) {
          if (indices[p] == i) {
            already_picked = true;
            break;
          }
        }
        if (already_picked)
          continue;
        if (processed_data[i].magnitude_G > best_mag) {
          best_mag = processed_data[i].magnitude_G;
          indices[pick] = i;
        }
      }
    }
    break;
  }
  }

  // For the case of 3+ valid emags, create the 3 possible combination pairs
  calculated_positions[0].emag_index_0 = indices[0];
  calculated_positions[0].emag_index_1 = indices[1];
  calculated_positions[1].emag_index_0 = indices[0];
  calculated_positions[1].emag_index_1 = indices[2];
  calculated_positions[2].emag_index_0 = indices[1];
  calculated_positions[2].emag_index_1 = indices[2];
  return true;
}

static bool solvePositions(CalculatedPosition *calculated_positions,
                           ProcessedEmagData *processed_data) {
  bool any_solved = false;

  for (uint8_t i = 0; i < 3; i++) {
    if (calculated_positions[i].emag_index_0 == 0xFF ||
        calculated_positions[i].emag_index_1 == 0xFF) {
      continue;
    }
    uint8_t idx0 = calculated_positions[i].emag_index_0;
    uint8_t idx1 = calculated_positions[i].emag_index_1;

    float e0x = emag_positions[idx0][0];
    float e0y = emag_positions[idx0][1];
    float e1x = emag_positions[idx1][0];
    float e1y = emag_positions[idx1][1];

    float m0 = processed_data[idx0].magnitude_G;
    float m1 = processed_data[idx1].magnitude_G;
    float a0 = processed_data[idx0].azimuth_angle_rad;
    float a1 = processed_data[idx1].azimuth_angle_rad;

    // Distance ratio from dipole falloff: B ∝ 1/r³  →  d0/d1 = (m1/m0)^(1/3)
    float ratio = cbrtf(m1 / m0);

    float da = a0 - a1; // Δα = α0 - α1
    float P = ratio * cosf(da) - 1.0f;
    float Q = ratio * sinf(da);
    float denom = P * P + Q * Q;

    if (denom < 1e-6f) {
      ESP_LOGD(TAG, "  pair[%u]: degenerate geometry (P^2+Q^2=%.6f), skipped",
               i, denom);
      Serial.printf("%u,%u,%u,%.2f,%.2f,%.2f,%.2f,%.4f,%.4f,%.4f,%.4f,%.4f,%."
                    "4f,%.4f,%.4f,%.6f,,,,,,,,\n",
                    i, idx0, idx1, e0x, e0y, e1x, e1y, m0, m1, a0, a1, ratio,
                    da, P, Q, denom);
      continue;
    }

    float dex = e0x - e1x;
    float dey = e0y - e1y;
    float emag_sep_sq = dex * dex + dey * dey;
    float d1 = sqrtf(emag_sep_sq / denom);

    float A = dex / d1;
    float B = dey / d1;

    // β1 = direction from robot to emag1 in world frame
    float beta1 = atan2f(P * B - Q * A, P * A + Q * B);

    // Heading: β1 = α1 + θ  →  θ = β1 - α1
    float theta = beta1 - a1;
    // Normalize θ to [-π, π]
    theta = fmodf(theta + (float)M_PI, 2.0f * (float)M_PI);
    if (theta < 0.0f)
      theta += 2.0f * (float)M_PI;
    theta -= (float)M_PI;

    float rx = e1x - d1 * cosf(beta1);
    float ry = e1y - d1 * sinf(beta1);

    calculated_positions[i].pos_x_mm = rx;
    calculated_positions[i].pos_y_mm = ry;
    calculated_positions[i].ang_rad = theta;
    calculated_positions[i].confidence = fminf(m0, m1) * sqrtf(denom);
    any_solved = true;

    // Serial.printf("%u,%u,%u,%.2f,%.2f,%.2f,%.2f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.6f,%.2f,%.4f,%.4f,%.4f,%.4f,%.2f,%.2f,%.4f\n",
    //               i, idx0, idx1, e0x, e0y, e1x, e1y, m0, m1, a0, a1, ratio,
    //               da, P, Q, denom, d1, A, B, beta1, theta, rx, ry,
    //               calculated_positions[i].confidence);
  }

  return any_solved;
}

static bool computePositionFromFrameData(const EmagFrameData *frame,
                                         Robot *robot) {
  uint32_t calc_start = micros();

  ESP_LOGD(TAG, "========== EMAG FRAME DATA ==========");
  ProcessedEmagData processed_data[EMAG_COUNT];
  uint8_t valid_emag_count = processEmagReadings(frame, processed_data);
  ESP_LOGD(TAG, "Valid electromagnet readings: %u / %u", valid_emag_count,
           EMAG_COUNT);

  ESP_LOGD(TAG, "========== IDENTIFY VALID EMAGS ==========");
  CalculatedPosition calculated_positions[3];
  populateEmagIndices(calculated_positions, processed_data, valid_emag_count);

  ESP_LOGD(TAG, "========== SOLVE POSITIONS ==========");
  bool solutions_exist = solvePositions(calculated_positions, processed_data);

  if (solutions_exist) {
    // Confidence-weighted average of all solved pairs
    float sum_conf = 0.0f;
    float sum_x = 0.0f, sum_y = 0.0f;
    // For angular averaging, use sin/cos decomposition to handle wrap-around
    float sum_sin = 0.0f, sum_cos = 0.0f;

    for (uint8_t i = 0; i < 3; i++) {
      float c = calculated_positions[i].confidence;
      if (c <= 0.0f)
        continue;
      sum_conf += c;
      sum_x += c * calculated_positions[i].pos_x_mm;
      sum_y += c * calculated_positions[i].pos_y_mm;
      sum_sin += c * sinf(calculated_positions[i].ang_rad);
      sum_cos += c * cosf(calculated_positions[i].ang_rad);

      ESP_LOGD(
          TAG, "  pair[%u]: pos=(%.1f, %.1f) mm  theta=%.2f rad  conf=%.3f", i,
          calculated_positions[i].pos_x_mm, calculated_positions[i].pos_y_mm,
          calculated_positions[i].ang_rad, c);
    }

    float inv_conf = 1.0f / sum_conf;
    float sensor_x = sum_x * inv_conf;
    float sensor_y = sum_y * inv_conf;
    float sensor_theta = atan2f(sum_sin, sum_cos);

    float robot_x, robot_y, robot_theta;
    sensorToRobotFrame(sensor_x, sensor_y, sensor_theta, &robot_x, &robot_y,
                       &robot_theta);
    robot->setTruePose(robot_x, robot_y, robot_theta, sum_conf);
  }

  // Report position calculation time, this is bad if longer than frame length
  uint32_t calc_end = micros();
  ESP_LOGD(TAG, "Calc time: %lu us", calc_end - calc_start);
  return solutions_exist;
}

static void sampleTimerCallback(void *arg) {
  // Runs in esp_timer task context — use direct notify (no ISR variant needed)
  xTaskNotifyGive(s_sensor_task_handle);
}

void PositionEstimator_SetSyncTime(int64_t sync_time_us) {
  sync_pulse_time_us = sync_time_us + EMAG_SAMPLE_TIME_US;
  next_frame_time_us = sync_pulse_time_us;
  synced = true;
  current_state = STATE_IDLE;
  ESP_LOGD(TAG, "Sync time: {%lu} us", sync_time_us);
}

void PositionEstimator_SensorTask(void *pvParameters) {
  vTaskDelay(pdMS_TO_TICKS(100));

  if (emag_frame_queue == NULL) {
    ESP_LOGE(TAG, "emag_frame_queue not initialized");
    vTaskDelete(NULL);
    return;
  }

  if (!mag.enableContinuousMode()) {
    ESP_LOGE(TAG, "Failed to enable continuous mode");
    vTaskDelete(NULL);
    return;
  }

  s_sensor_task_handle = xTaskGetCurrentTaskHandle();

  esp_timer_create_args_t timer_args = {};
  timer_args.callback = sampleTimerCallback;
  timer_args.dispatch_method = ESP_TIMER_TASK;
  timer_args.name = "mag_sample";
  if (esp_timer_create(&timer_args, &s_sample_timer) != ESP_OK) {
    ESP_LOGE(TAG, "Failed to create mag sample timer");
    vTaskDelete(NULL);
    return;
  }

  const int64_t slot_us = 1000LL * (EMAG_FWD_ON_TIME_MS + EMAG_REV_ON_TIME_MS);
  bool set_reset_done = false;

  // Performance counters — logged per frame
  uint32_t reads_attempted = 0;
  uint32_t reads_stale = 0;

  while (1) {
    // ---- Block until timer notification (MEASURING) or timeout (IDLE) ----
    if (current_state == STATE_MEASURING) {
      // Timer drives precise wakeups; 5 ms is a safety-net fallback only
      ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(5));
    } else {
      // Timer is stopped in IDLE/SYNC_LOST — use timeout to service idle work
      int64_t now_us = esp_timer_get_time();
      int64_t idle_wait_us =
          synced ? (next_frame_time_us - now_us - 1000LL) : 10000LL;
      if (idle_wait_us < 1000LL)
        idle_wait_us = 1000LL;
      if (idle_wait_us > 10000LL)
        idle_wait_us = 10000LL;
      ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(idle_wait_us / 1000));
    }

    if (!mag_initialized)
      continue;

    int64_t current_micros = esp_timer_get_time();

    if (current_state == STATE_MEASURING) {
      reads_attempted++;
      if (!mag.readMeasurement()) {
        // Busy-wait up to 3 × 50 µs for data to become ready
        bool ready = false;
        for (int retry = 0; retry < 3; retry++) {
          esp_rom_delay_us(50);
          reads_stale++;
          if (mag.readMeasurement()) {
            ready = true;
            break;
          }
        }
        if (!ready) {
          ESP_LOGD(TAG,
                   "Stale read unresolved after retries (total stale: %lu)",
                   reads_stale);
          continue;
        }
      }

      // Invert y and z to account for sensor orientation
      float mx = mag.getFieldGaussX();
      float my = -mag.getFieldGaussY();
      float mz = -mag.getFieldGaussZ();

      avgX.add(mx);
      avgY.add(my);
      avgZ.add(mz);

      ESP_LOGD(
          TAG,
          "Raw mag: X=%.3f Y=%.3f Z=%.3f  |  Avg mag: X=%.3f Y=%.3f Z=%.3f", mx,
          my, mz, avgX.avg(), avgY.avg(), avgZ.avg());

      int64_t frame_elapsed_us = current_micros - frame_start_time_us;
      uint8_t emag_index = (uint8_t)(frame_elapsed_us / slot_us);

      // Serial.printf("%ld us  \tmx: %.3f\tmy: %.3f\tmz: %.3f\n",
      // frame_elapsed_us, mx, my, mz);

      if (emag_index >= EMAG_COUNT) {
        // Frame complete — stop the periodic timer and return to IDLE
        esp_timer_stop(s_sample_timer);
        float miss_pct = (reads_attempted > 0)
                             ? (100.0f * reads_stale / reads_attempted)
                             : 0.0f;
        ESP_LOGI(TAG, "Frame complete: %lu reads, %lu stale (%.1f%% miss)",
                 reads_attempted, reads_stale, miss_pct);
        reads_attempted = 0;
        reads_stale = 0;

        current_frame.bg_x = avgX.avg();
        current_frame.bg_y = avgY.avg();
        current_frame.bg_z = avgZ.avg();
        xQueueOverwrite(emag_frame_queue, &current_frame);
        current_state = STATE_IDLE;
        set_reset_done = false;

        next_frame_time_us =
            sync_pulse_time_us + ((current_micros - sync_pulse_time_us) /
                                      (EMAG_FRAME_LEN_MS * 1000) +
                                  1) *
                                     (EMAG_FRAME_LEN_MS * 1000);
        continue;
      }

      // Periodic timer keeps running — no re-arm needed
      int64_t offset_us = frame_elapsed_us % slot_us;

      // Forward active window (trim leading and trailing edges)
      bool in_fwd =
          (offset_us >= EMAG_TRIM_MS * 1000) &&
          (offset_us < EMAG_FWD_ON_TIME_MS * 1000 - EMAG_TRIM_MS * 1000);

      // Reverse active window
      bool in_rev =
          (offset_us >= (EMAG_FWD_ON_TIME_MS + EMAG_TRIM_MS) * 1000) &&
          (offset_us <
           (EMAG_FWD_ON_TIME_MS + EMAG_REV_ON_TIME_MS - EMAG_TRIM_MS) * 1000);

      if (in_fwd || in_rev) {
        EmagReading *r = &current_frame.readings[emag_index];
        if (r->count < MAX_SAMPLES_PER_EMAG) {
          r->x[r->count] = mx;
          r->y[r->count] = my;
          r->z[r->count] = mz;
          r->is_forward[r->count] = in_fwd;
          r->timestamp_us[r->count] = current_micros - frame_start_time_us;
          r->count++;
        }
      }
    } else {
      // ---- IDLE / SYNC_LOST: read mag at background rate to maintain rolling
      // averages ----
      if (mag.readMeasurement()) {
        float mx = mag.getFieldGaussX();
        float my = -mag.getFieldGaussY();
        float mz = -mag.getFieldGaussZ();
        avgX.add(mx);
        avgY.add(my);
        avgZ.add(mz);
      }

      if (current_state == STATE_IDLE) {
        int64_t time_to_next_frame_us = next_frame_time_us - current_micros;
        if (synced && time_to_next_frame_us <= 0) {
          frame_start_time_us = next_frame_time_us;
          memset(&current_frame, 0, sizeof(current_frame));
          current_state = STATE_MEASURING;
          esp_timer_start_periodic(s_sample_timer, EMAG_MIN_SAMPLE_PERIOD_US);
          ESP_LOGD(TAG,
                   "Starting new emag frame at %lld us, periodic timer started",
                   frame_start_time_us);
        } else {
          int64_t delay_ms = (time_to_next_frame_us / 1000) - 1;
          // TODO make sure SR is run every 1s even if no state changes
          if (!set_reset_done && delay_ms > 5) {
            if (!mag.setReset()) {
              ESP_LOGW(TAG, "Magnetometer set/reset failed");
            }
            set_reset_done = true;
          }
        }
      }
    }
  }

  esp_timer_stop(s_sample_timer);
  esp_timer_delete(s_sample_timer);
  mag.disableContinuousMode();
}

void PositionEstimator_CalcTask(void *pvParameters) {
  Robot *robot = (Robot *)pvParameters;

  if (robot == NULL || emag_frame_queue == NULL) {
    ESP_LOGE(TAG, "PositionEstimator_CalcTask bad parameters");
    vTaskDelete(NULL);
    return;
  }

  EmagFrameData frame;

  while (1) {
    if (xQueueReceive(emag_frame_queue, &frame, portMAX_DELAY) == pdTRUE) {
      ESP_LOGD(TAG,
               "================ NEW EMAG FRAME RECEIVED ================");
      if (!computePositionFromFrameData(&frame, robot)) {
        ESP_LOGD(TAG, "Position computation failed for frame");
      }
      ESP_LOGD(TAG, "================ END OF FRAME ================");
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}
