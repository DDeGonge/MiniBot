#include "robot.h"
#include <Arduino.h>
#include <cmath>
#undef LOG_LOCAL_LEVEL
#define LOG_LOCAL_LEVEL LOG_LEVEL_ROBOT
#include "config.h"
#include "esp_log.h"

static const char *TAG = "ROBOT";

static Robot *g_motor_test_robot = NULL;
static bool motor_test_m0_running = false;
static bool motor_test_m1_running = false;

bool Robot::initialize() {
  if (!left_wheel.initialize(L_WHEEL_STEP_PIN, L_WHEEL_DIR_PIN, STEPPER_EN_PIN,
                             STEPPER_RST_PIN, L_WHEEL_REVERSE)) {
    return false;
  }

  if (!right_wheel.initialize(R_WHEEL_STEP_PIN, R_WHEEL_DIR_PIN, STEPPER_EN_PIN,
                              STEPPER_RST_PIN, R_WHEEL_REVERSE)) {
    return false;
  }

  left_wheel.setMicrostepping(MSET_STEP_LVL, MSET_DIR_LVL);
  right_wheel.setMicrostepping(MSET_STEP_LVL, MSET_DIR_LVL);
  left_wheel.resetDriver(); // only one reset needed, reset pin is shared

  wheel_radius_mm = WHEEL_RADIUS_MM;
  wheel_spacing_mm = WHEEL_SPACING_MM;

  steps_per_revolution = STEPS_PER_REVOLUTION;
  robot_max_velocity_mm_s = ROBOT_MAX_VELOCITY_MM_S;
  robot_max_accel_mm_s2 = ROBOT_MAX_ACCEL_MM_S2;
  max_rot_vel_rad_s = MAX_ROT_VEL_RAD_S;
  max_rot_accel_rad_s2 = MAX_ROT_ACCEL_RAD_S2;
  stepper_max_velocity_mm_s = STEPPER_MAX_VELOCITY_MM_S;

  positionX = 0.0f;
  positionY = 0.0f;
  orientation = 0.0f;

  true_x = 0.0f;
  true_y = 0.0f;
  true_theta = 0.0f;
  true_pose_mutex = xSemaphoreCreateMutex();

  battery_voltage = 0.0f;
  system_status = 0;
  is_moving = false;

  return true;
}

void Robot::updateEstimatedPosition() {
  // TODO: Implement odometry calculations based on step counts
  // This will convert stepper motor step counts to X,Y position and orientation
  // Updates true_x, true_y, and true_theta continuously
}

void Robot::setTruePose(float x, float y, float theta, float confidence) {
  // if (is_moving) {
  //     // Don't update true pose while actively moving, steppers induce too
  //     much noise return;
  // }

  uint32_t now_us = micros();
  xSemaphoreTake(true_pose_mutex, portMAX_DELAY);

  if (!true_pose_initialized) {
    true_x = x;
    true_y = y;
    true_theta = theta;
    true_pose_initialized = true;
    true_pose_last_update_us = now_us;
    xSemaphoreGive(true_pose_mutex);
    return;
  }

  float dt_s = (float)(now_us - true_pose_last_update_us) * 1e-6f;
  true_pose_last_update_us = now_us;

  // Compute alpha from cutoff frequency: alpha = 1 - exp(-2*pi*f_c*dt)
  // Scale by confidence: high confidence → full response, low → heavier
  // smoothing
  float conf_scale = fminf(confidence / TRUE_POSE_LPF_REF_CONFIDENCE, 1.0f);
  if (conf_scale < 0.1f)
    conf_scale = 0.1f;
  const float alpha =
      (1.0f - expf(-2.0f * M_PI * TRUE_POSE_LPF_CUTOFF_HZ * dt_s)) * conf_scale;
  true_x = alpha * x + (1.0f - alpha) * true_x;
  true_y = alpha * y + (1.0f - alpha) * true_y;

  // For theta, filter on the shortest angular delta to handle wrap-around
  float delta = theta - true_theta;
  while (delta > M_PI)
    delta -= 2.0f * M_PI;
  while (delta < -M_PI)
    delta += 2.0f * M_PI;
  true_theta += alpha * delta;
  while (true_theta > M_PI)
    true_theta -= 2.0f * M_PI;
  while (true_theta < -M_PI)
    true_theta += 2.0f * M_PI;

  // TODO for now just using this as truth, eventually be smarter about when to
  // use dead reckoning vs sensed
  updatePositionFromTruePose();

  xSemaphoreGive(true_pose_mutex);
}

bool Robot::getTruePose(float *x, float *y, float *orientation) {
  xSemaphoreTake(true_pose_mutex, portMAX_DELAY);

  if (!true_pose_initialized) {
    xSemaphoreGive(true_pose_mutex);
    return false;
  }

  uint32_t age_ms = (micros() - true_pose_last_update_us) / 1000;
  if (age_ms > TRUE_POSE_STALE_TIMEOUT_MS) {
    xSemaphoreGive(true_pose_mutex);
    return false;
  }

  if (x != NULL)
    *x = true_x;
  if (y != NULL)
    *y = true_y;
  if (orientation != NULL)
    *orientation = true_theta;
  xSemaphoreGive(true_pose_mutex);
  return true;
}

void Robot::updatePositionFromTruePose() {
  positionX = true_x;
  positionY = true_y;
  orientation = true_theta;
}

void Robot::getPosition(float *x, float *y, float *orientation) {
  if (x != NULL)
    *x = positionX;
  if (y != NULL)
    *y = positionY;
  if (orientation != NULL)
    *orientation = this->orientation;
}

void Robot::setPosition(float x, float y, float orientation) {
  positionX = x;
  positionY = y;
  this->orientation = orientation;
}

Robot::MotionProfile Robot::calculateMotionProfile(float distance_mm,
                                                   float max_velocity_mm_s) {
  float accel_time_s = max_velocity_mm_s / robot_max_accel_mm_s2;
  float accel_distance_mm =
      0.5f * robot_max_accel_mm_s2 * accel_time_s * accel_time_s;
  bool full_profile = (distance_mm >= 2.0f * accel_distance_mm);

  return {accel_time_s, accel_time_s * 1000.0f, full_profile};
}

float Robot::calculateVelocityAtTime(float elapsed_ms, float remaining_ms,
                                     float accel_phase_ms,
                                     float max_velocity_mm_s,
                                     bool full_profile) {
  if (!full_profile)
    return max_velocity_mm_s;

  if (elapsed_ms < accel_phase_ms) {
    return robot_max_accel_mm_s2 * (elapsed_ms / 1000.0f);
  } else if (remaining_ms < accel_phase_ms) {
    return robot_max_accel_mm_s2 * (remaining_ms / 1000.0f);
  }
  return max_velocity_mm_s;
}

float Robot::calculateMaxVelocityForTime(float distance_mm, float target_time_s,
                                         float max_accel) {
  // Solves for max velocity given distance and time constraints
  // Triangular profile: d = a*t²/2, so v_max = a*t/2
  float min_time_triangular = 2.0f * sqrt(distance_mm / max_accel);

  if (target_time_s <= min_time_triangular) {
    return sqrt(distance_mm * max_accel);
  }

  // Trapezoidal profile: d = v_max * (t - v_max/a)
  // Solving: v_max = a*t/2 - sqrt((a*t/2)² - a*d)
  float half_at = max_accel * target_time_s / 2.0f;
  float discriminant = half_at * half_at - max_accel * distance_mm;

  if (discriminant < 0) {
    return sqrt(distance_mm * max_accel);
  }

  float v_max = half_at - sqrt(discriminant);

  if (v_max <= 0) {
    return sqrt(distance_mm * max_accel);
  }

  return v_max;
}

void Robot::executeMotionLoop(int32_t total_steps, float base_step_time_us,
                              float max_velocity_mm_s,
                              const MotionProfile &profile,
                              TickType_t start_tick, TickType_t target_end_tick,
                              std::function<void(int32_t)> step_callback) {
  ESP_LOGD(TAG,
           "    executeMotionLoop: total_steps=%ld, base_step_time=%.2f us, "
           "max_vel=%.2f mm/s",
           total_steps, base_step_time_us, max_velocity_mm_s);
  ESP_LOGD(TAG, "    Profile: accel_phase=%.2f ms, full_profile=%s",
           profile.accel_phase_ms, profile.full_profile ? "YES" : "NO");

  TickType_t current_tick;
  float elapsed_ms, remaining_ms, current_velocity_mm_s;
  uint32_t step_delay_us;
  TickType_t step_delay_ticks;

  for (int32_t step_index = 0; step_index < total_steps; step_index++) {
    current_tick = xTaskGetTickCount();
    elapsed_ms = (float)(current_tick - start_tick) * portTICK_PERIOD_MS;
    remaining_ms = (float)(target_end_tick - current_tick) * portTICK_PERIOD_MS;

    current_velocity_mm_s = calculateVelocityAtTime(
        elapsed_ms, remaining_ms, profile.accel_phase_ms, max_velocity_mm_s,
        profile.full_profile);

    step_delay_us = (uint32_t)fmax(1.0f, base_step_time_us * max_velocity_mm_s /
                                             current_velocity_mm_s);
    step_delay_ticks = pdMS_TO_TICKS(fmax(1, (uint32_t)(step_delay_us / 1000)));

    step_callback(step_index);

    if (step_delay_ticks > 0) {
      vTaskDelay(step_delay_ticks);
    } else {
      vTaskDelay(1);
    }
  }
}

// Normalize angle to [-PI, PI]
static float normalizeAngle(float angle) {
  while (angle > M_PI)
    angle -= 2.0f * M_PI;
  while (angle < -M_PI)
    angle += 2.0f * M_PI;
  return angle;
}

// ============================================================================
// Wheel Motion Profile
// ============================================================================

// Calculate motion profile for a single wheel given distance and target time
// Acceleration is fixed. Velocity is adjusted to achieve target time.
// Profile will be triangular (accel->decel) or trapezoidal
// (accel->cruise->decel)
static Robot::WheelMotion calculateWheelProfile(float distance_mm,
                                                float target_time_s,
                                                float max_vel_limit,
                                                float accel_limit,
                                                float steps_per_mm) {
  Robot::WheelMotion m = {};
  m.distance_mm = fabs(distance_mm);
  m.forward = (distance_mm >= 0);
  m.accel_mm_s2 = accel_limit;
  m.total_steps = (int32_t)round(m.distance_mm * steps_per_mm);

  if (m.distance_mm < 0.01f || m.total_steps == 0) {
    m.total_time_s = 0;
    return m;
  }

  float d = m.distance_mm;
  float a = accel_limit;
  float v_limit = max_vel_limit;
  float v_triangular_peak = sqrt(a * d);

  float min_time;
  if (v_triangular_peak <= v_limit) {
    // Triangular profile achieves minimum time (doesn't hit velocity limit)
    // t_min = 2 * sqrt(d / a)
    min_time = 2.0f * sqrt(d / a);
  } else {
    // Trapezoidal at max velocity achieves minimum time
    // t_min = v/a + d/v (accel time + cruise time, where cruise includes decel)
    min_time = v_limit / a + d / v_limit;
  }

  float effective_time = fmax(target_time_s, min_time);
  float half_at = a * effective_time / 2.0f;
  float discriminant = half_at * half_at - a * d;

  if (discriminant < 0.0001f) {
    // No real solution or at boundary - use triangular profile
    m.max_velocity_mm_s = v_triangular_peak;
  } else {
    // Trapezoidal: take smaller root for slower velocity
    m.max_velocity_mm_s = half_at - sqrt(discriminant);
  }

  m.max_velocity_mm_s = fmax(fmin(m.max_velocity_mm_s, v_limit), 0.1f);
  m.accel_time_s = m.max_velocity_mm_s / a;
  float accel_distance = 0.5f * a * m.accel_time_s * m.accel_time_s;

  if (d >= 2.0f * accel_distance + 0.001f) {
    // Trapezoidal profile
    float cruise_distance = d - 2.0f * accel_distance;
    m.cruise_time_s = cruise_distance / m.max_velocity_mm_s;
    m.is_triangular = false;
  } else {
    // Triangular profile
    m.accel_time_s = sqrt(d / a);
    m.max_velocity_mm_s = a * m.accel_time_s;
    m.cruise_time_s = 0;
    m.is_triangular = true;
  }

  m.total_time_s = 2.0f * m.accel_time_s + m.cruise_time_s;

  ESP_LOGD(TAG,
           "    WheelProfile: d=%.1f mm, target=%.0f ms, actual=%.0f ms, "
           "v=%.1f mm/s, %s",
           d, target_time_s * 1000, m.total_time_s * 1000, m.max_velocity_mm_s,
           m.is_triangular ? "TRI" : "TRAP");

  return m;
}

// Get velocity at a given time for a wheel profile
static float getVelocityAtTime(const Robot::WheelMotion &m, float t) {
  if (t < 0 || m.total_time_s <= 0)
    return 0;
  if (t > m.total_time_s)
    return 0;

  float decel_start = m.accel_time_s + m.cruise_time_s;

  if (t < m.accel_time_s) {
    // Acceleration phase
    return m.accel_mm_s2 * t;
  } else if (t < decel_start) {
    // Cruise phase
    return m.max_velocity_mm_s;
  } else {
    // Deceleration phase
    float t_decel = t - decel_start;
    return fmax(0.0f, m.max_velocity_mm_s - m.accel_mm_s2 * t_decel);
  }
}

static bool preciseWaitUntil(int64_t target_us) {
  int64_t now_us = esp_timer_get_time();
  int64_t wait_us = target_us - now_us;

  // Yielding wait if longer than 1.5ms
  if (wait_us > 1500) {
    vTaskDelay(pdMS_TO_TICKS((uint32_t)((wait_us - 500) / 1000)));
  }

  now_us = esp_timer_get_time();
  wait_us = target_us - now_us;
  if (wait_us < 0) {
    return false; // Target time already passed
  }
  delayMicroseconds((uint32_t)wait_us);
  return true;
}

// ============================================================================
// Main Motion Control
// ============================================================================

void Robot::setTargetPose(MotionCommand target) {
  is_moving = true;
  if (motor_test_active) {
    ESP_LOGW(TAG, "Received motion command while motor test active. Stopping");
    stopMotorTest();
    return;
  }

  float target_x = target.target_position_x_mm;
  float target_y = target.target_position_y_mm;
  float target_theta = normalizeAngle(target.target_orientation_rad);
  float move_duration_s = target.move_duration_ms / 1000.0f;

  float current_x = positionX;
  float current_y = positionY;
  float current_theta = normalizeAngle(orientation);

  // Calculate deltas
  float dx = target_x - current_x;
  float dy = target_y - current_y;
  float linear_distance = sqrt(dx * dx + dy * dy);
  float angle_delta = normalizeAngle(target_theta - current_theta);

  ESP_LOGD(TAG, "\n=== Motion Command ===");
  ESP_LOGD(TAG, "From: (%.1f, %.1f, %.2f rad)", current_x, current_y,
           current_theta);
  ESP_LOGD(TAG, "To:   (%.1f, %.1f, %.2f rad)", target_x, target_y,
           target_theta);
  ESP_LOGD(TAG, "Delta: d=%.1f mm, a=%.2f rad, t=%.0f ms", linear_distance,
           angle_delta, move_duration_s * 1000);

  // Enable motors
  left_wheel.enable();
  right_wheel.enable();

  // Classify motion type
  MotionType motion_type = MotionType::NONE;

  bool position_match = (linear_distance < POSITION_TOLERANCE_MM);
  bool angle_match = (fabs(angle_delta) < ANGLE_TOLERANCE_RAD);

  if (position_match && angle_match) {
    // Already at target
    motion_type = MotionType::NONE;
    ESP_LOGD(TAG, "Type: NONE (already at target)");
  } else if (position_match) {
    // Only rotation needed
    motion_type = MotionType::ROTATION_ONLY;
    ESP_LOGD(TAG, "Type: ROTATION_ONLY");
  } else {
    // Position change required - check if straight line is possible
    float heading_to_target = atan2(dy, dx);
    float heading_error = normalizeAngle(heading_to_target - current_theta);

    // Straight line possible if heading matches (forward or backward)
    if (fabs(heading_error) < ANGLE_TOLERANCE_RAD) {
      motion_type = MotionType::STRAIGHT_LINE;
      ESP_LOGD(TAG, "Type: STRAIGHT_LINE (forward)");
    } else if (fabs(fabs(heading_error) - M_PI) < ANGLE_TOLERANCE_RAD) {
      motion_type = MotionType::STRAIGHT_LINE;
      ESP_LOGD(TAG, "Type: STRAIGHT_LINE (backward)");
    } else {
      motion_type = MotionType::ARC_THEN_ROTATE;
      ESP_LOGD(TAG, "Type: ARC_THEN_ROTATE");
    }
  }

  // Execute motion
  uint32_t start_time = millis();

  switch (motion_type) {
  case MotionType::NONE:
    break;

  case MotionType::ROTATION_ONLY:
    executeRotation(angle_delta, move_duration_s);
    break;

  case MotionType::STRAIGHT_LINE: {
    float heading_to_target = atan2(dy, dx);
    float heading_error = normalizeAngle(heading_to_target - current_theta);
    bool backward = (fabs(heading_error) > M_PI / 2.0f);
    float signed_distance = backward ? -linear_distance : linear_distance;

    // Check if we need final rotation
    float final_rotation = angle_delta;
    if (fabs(final_rotation) > ANGLE_TOLERANCE_RAD) {
      // Allocate 85% to straight, 15% to rotation
      float straight_time = move_duration_s * 0.85f;
      float rotate_time = move_duration_s * 0.15f;
      executeStraightLine(signed_distance, straight_time);
      executeRotation(final_rotation, rotate_time);
    } else {
      executeStraightLine(signed_distance, move_duration_s);
    }
    break;
  }

  case MotionType::ARC_THEN_ROTATE: {
    // Execute arc from current orientation to target position
    // Then rotate in place to match target angle
    executeArcToPosition(dx, dy, current_theta, move_duration_s, angle_delta);
    break;
  }
  }

  // maybe temporary, wait 5ms to allow motors to stabilize before disabling
  vTaskDelay(pdMS_TO_TICKS(5));

  uint32_t elapsed = millis() - start_time;
  ESP_LOGD(TAG, "Motion complete in %lu ms", elapsed);

  // Update position
  positionX = target_x;
  positionY = target_y;
  orientation = target_theta;

  left_wheel.disable();
  right_wheel.disable();
  is_moving = false;
}

// ============================================================================
// Shared Accel / Cruise(hw timer) / Decel Step Loop
// Drives left_wheel and right_wheel synchronously through a single WheelMotion
// profile. Both wheels must have direction set before calling.
// ============================================================================

void Robot::executeStepLoop(const WheelMotion &profile, float steps_per_mm) {
  const float step_distance_mm = 1.0f / steps_per_mm;
  const float a = profile.accel_mm_s2;
  const float v_max = profile.max_velocity_mm_s;

  float accel_dist = 0.5f * a * profile.accel_time_s * profile.accel_time_s;
  int32_t accel_steps = (int32_t)(accel_dist * steps_per_mm);
  int32_t decel_steps = accel_steps;
  int32_t cruise_steps = profile.total_steps - accel_steps - decel_steps;
  if (cruise_steps < 0)
    cruise_steps = 0;

  int32_t steps_done = 0;
  uint8_t phase = 0;
  int64_t start_us = esp_timer_get_time();

  if (cruise_steps != 0) {
    float vel_rad_s = v_max / wheel_radius_mm;
    float steps_per_rad_val = steps_per_revolution / (2.0f * M_PI);
    left_wheel.configureTimer(vel_rad_s, steps_per_rad_val, cruise_steps);
    right_wheel.configureTimer(vel_rad_s, steps_per_rad_val, cruise_steps);
  }

  ESP_LOGD(
      TAG,
      "step_dist=%.3f mm, accel_steps=%ld, cruise_steps=%ld, decel_steps=%ld\n",
      step_distance_mm, accel_steps, cruise_steps, decel_steps);

  while (steps_done < profile.total_steps) {
    if (phase == 0 && steps_done >= accel_steps) {
      phase = (cruise_steps > 0) ? 1 : 2;
    }

    int64_t now_us = esp_timer_get_time();
    float elapsed_s = (float)(now_us - start_us) * 1e-6f;

    if (elapsed_s > profile.total_time_s + 1.0f)
      break;

    int64_t target_us = -1;

    switch (phase) {
    case 0: {
      // Acceleration: analytical step time from rest
      float t_next = sqrtf(2.0f * (steps_done + 1) * step_distance_mm / a);
      target_us = start_us + (int64_t)(t_next * 1e6f);
      break;
    }

    case 1: {
      // Cruise: Start timers and wait for cruise to complete
      left_wheel.startTimer();
      right_wheel.startTimer();
      ESP_LOGD(TAG,
               "  Cruise phase started at step %ld, target velocity %.1f mm/s",
               steps_done, v_max);

      // Wait until close to end of cruise phase
      int64_t cruise_end_us =
          start_us +
          (int64_t)((profile.accel_time_s + profile.cruise_time_s) * 1e6f) -
          500;
      if (!preciseWaitUntil(cruise_end_us)) {
        ESP_LOGW(TAG, "  Cruise phase timing missed");
      }

      // Then busywait until both timers have completed, 2ms timeout
      while (esp_timer_get_time() < cruise_end_us + 2000) {
        if (!left_wheel.isTimerRunning() && !right_wheel.isTimerRunning()) {
          ESP_LOGD(TAG, "  Cruise phase timer completed");
          break;
        }
      }

      steps_done += cruise_steps;
      phase = 2;
      break;
    }

    case 2: {
      // Deceleration: mirror of accel, time from end
      int32_t remaining_after = profile.total_steps - steps_done - 1;
      float t_from_end = sqrtf(2.0f * remaining_after * step_distance_mm / a);

      int64_t end_us = start_us + (int64_t)(profile.total_time_s * 1e6f);
      int64_t next_step_us = end_us - (int64_t)(t_from_end * 1e6f);
      int64_t time_to_next = next_step_us - now_us;
      if (time_to_next > 50000)
        time_to_next = 50000;
      target_us = now_us + time_to_next;
      break;
    }
    }

    if (target_us != -1) {
      if (!preciseWaitUntil(target_us)) {
        ESP_LOGW(TAG, "  Accel/Decel phase timing missed for step %ld",
                 steps_done + 1);
      }
      left_wheel.step();
      right_wheel.step();
      steps_done++;
    }
  }
}

// ============================================================================
// Execute Rotation (in place)
// ============================================================================

void Robot::executeRotation(float angle_rad, float target_time_s) {
  if (fabs(angle_rad) < 0.001f)
    return;

  // Choose shorter rotation direction
  angle_rad = normalizeAngle(angle_rad);

  ESP_LOGD(TAG, "  Rotation: %.2f rad (%.1f deg) in %.0f ms", angle_rad,
           angle_rad * 180.0f / M_PI, target_time_s * 1000);

  // Arc length each wheel travels (opposite directions)
  float wheel_arc = (wheel_spacing_mm / 2.0f) * fabs(angle_rad);
  float steps_per_mm = steps_per_revolution / (2.0f * M_PI * wheel_radius_mm);

  WheelMotion profile =
      calculateWheelProfile(wheel_arc, target_time_s, stepper_max_velocity_mm_s,
                            robot_max_accel_mm_s2, steps_per_mm);

  if (profile.total_steps == 0)
    return;

  // Positive angle = CCW = left backward, right forward
  bool cw = (angle_rad < 0);
  left_wheel.setDirection(cw);
  right_wheel.setDirection(!cw);

  ESP_LOGD(TAG, "  Profile: v=%.1f mm/s, t=%.0f ms, steps=%ld",
           profile.max_velocity_mm_s, profile.total_time_s * 1000,
           profile.total_steps);

  executeStepLoop(profile, steps_per_mm);
}

// ============================================================================
// Execute Straight Line
// ============================================================================

void Robot::executeStraightLine(float distance_mm, float target_time_s) {
  if (fabs(distance_mm) < 0.1f)
    return;

  ESP_LOGD(TAG, "  Straight: %.1f mm in %.0f ms", distance_mm,
           target_time_s * 1000);

  float steps_per_mm = steps_per_revolution / (2.0f * M_PI * wheel_radius_mm);

  WheelMotion profile = calculateWheelProfile(
      fabs(distance_mm), target_time_s, stepper_max_velocity_mm_s,
      robot_max_accel_mm_s2, steps_per_mm);

  if (profile.total_steps == 0)
    return;

  bool forward = (distance_mm >= 0);
  left_wheel.setDirection(forward);
  right_wheel.setDirection(forward);

  ESP_LOGD(TAG, "  Profile: v=%.1f mm/s, t=%.0f ms, steps=%ld, %s",
           profile.max_velocity_mm_s, profile.total_time_s * 1000,
           profile.total_steps, forward ? "FWD" : "REV");

  executeStepLoop(profile, steps_per_mm);
}

// ============================================================================
// Execute Arc Motion to Position (with optional final rotation)
// ============================================================================

void Robot::executeArcToPosition(float dx, float dy, float current_theta,
                                 float target_time_s, float final_angle_delta) {
  float linear_distance = sqrt(dx * dx + dy * dy);

  if (linear_distance < 0.1f)
    return;

  // Calculate angle from current heading to target position
  float heading_to_target = atan2(dy, dx);
  float heading_error = normalizeAngle(heading_to_target - current_theta);

  // The robot will curve from current_theta toward target position
  // Arc geometry: robot starts facing current_theta, ends at target position
  // The arc angle is 2 * heading_error (it's the exterior angle of the isoceles
  // triangle)
  float arc_angle = 2.0f * heading_error;

  // After arc, robot will be facing: current_theta + arc_angle
  float ending_theta = normalizeAngle(current_theta + arc_angle);

  // Calculate remaining rotation needed after arc
  float rotation_after_arc = normalizeAngle(final_angle_delta - arc_angle);

  ESP_LOGD(TAG,
           "  ArcToPos: d=%.1f mm, heading_err=%.2f rad, arc_angle=%.2f rad",
           linear_distance, heading_error, arc_angle);
  ESP_LOGD(TAG, "  Will end facing %.2f rad, need rotation %.2f rad after",
           ending_theta, rotation_after_arc);

  // Calculate arc radius: R = d / (2 * sin(heading_error))
  // When heading_error is small, radius is large (nearly straight)
  float abs_heading = fabs(heading_error);
  float arc_radius;

  if (abs_heading < 0.01f) {
    // Nearly straight - just go straight
    bool backward = (fabs(heading_error) > M_PI / 2.0f);
    float signed_distance = backward ? -linear_distance : linear_distance;

    if (fabs(rotation_after_arc) > ANGLE_TOLERANCE_RAD) {
      float straight_time = target_time_s * 0.85f;
      float rotate_time = target_time_s * 0.15f;
      executeStraightLine(signed_distance, straight_time);
      executeRotation(rotation_after_arc, rotate_time);
    } else {
      executeStraightLine(signed_distance, target_time_s);
    }
    return;
  }

  arc_radius = linear_distance / (2.0f * sin(abs_heading));

  // Minimum arc radius check
  if (arc_radius < MIN_ARC_RADIUS_MM) {
    arc_radius = MIN_ARC_RADIUS_MM;
  }

  // Calculate wheel distances
  // For CCW (positive arc_angle): left wheel is inner, right is outer
  // For CW (negative arc_angle): right wheel is inner, left is outer
  float inner_radius = arc_radius - wheel_spacing_mm / 2.0f;
  float outer_radius = arc_radius + wheel_spacing_mm / 2.0f;
  float abs_arc_angle = fabs(arc_angle);
  float inner_distance = fmax(0.0f, inner_radius * abs_arc_angle);
  float outer_distance = outer_radius * abs_arc_angle;

  float left_distance, right_distance;
  bool left_forward = true, right_forward = true;

  if (arc_angle > 0) {
    // CCW: left is inner
    left_distance = inner_distance;
    right_distance = outer_distance;
  } else {
    // CW: right is inner
    left_distance = outer_distance;
    right_distance = inner_distance;
  }

  // Handle case where inner radius is negative (tight turn)
  if (inner_radius < 0) {
    if (arc_angle > 0) {
      left_distance = fabs(inner_radius) * abs_arc_angle;
      left_forward = false; // Inner wheel goes backward
    } else {
      right_distance = fabs(inner_radius) * abs_arc_angle;
      right_forward = false;
    }
  }

  ESP_LOGD(TAG, "  Arc r=%.1f, left=%.1f mm (%s), right=%.1f mm (%s)",
           arc_radius, left_distance, left_forward ? "fwd" : "rev",
           right_distance, right_forward ? "fwd" : "rev");

  // Allocate time for arc and optional rotation
  float arc_time_s = target_time_s;
  float rotation_time_s = 0;

  if (fabs(rotation_after_arc) > ANGLE_TOLERANCE_RAD) {
    arc_time_s = target_time_s * 0.85f;
    rotation_time_s = target_time_s * 0.15f;
  }

  float steps_per_mm = steps_per_revolution / (2.0f * M_PI * wheel_radius_mm);

  // Calculate profiles for each wheel
  WheelMotion left_profile = calculateWheelProfile(
      left_distance, arc_time_s, stepper_max_velocity_mm_s,
      robot_max_accel_mm_s2, steps_per_mm);
  WheelMotion right_profile = calculateWheelProfile(
      right_distance, arc_time_s, stepper_max_velocity_mm_s,
      robot_max_accel_mm_s2, steps_per_mm);

  // Use the longer wheel's time as reference
  float motion_time_s =
      fmax(left_profile.total_time_s, right_profile.total_time_s);

  // Recalculate profiles with synchronized time
  left_profile = calculateWheelProfile(left_distance, motion_time_s,
                                       stepper_max_velocity_mm_s,
                                       robot_max_accel_mm_s2, steps_per_mm);
  right_profile = calculateWheelProfile(right_distance, motion_time_s,
                                        stepper_max_velocity_mm_s,
                                        robot_max_accel_mm_s2, steps_per_mm);

  // Set wheel directions
  left_wheel.setDirection(left_forward);
  right_wheel.setDirection(right_forward);

  ESP_LOGD(TAG, "  Left:  v=%.1f mm/s, steps=%ld",
           left_profile.max_velocity_mm_s, left_profile.total_steps);
  ESP_LOGD(TAG, "  Right: v=%.1f mm/s, steps=%ld",
           right_profile.max_velocity_mm_s, right_profile.total_steps);

  // Execute with independent wheel timing
  uint32_t start_us = micros();
  float step_distance_mm = 1.0f / steps_per_mm;

  float left_distance_done = 0;
  float right_distance_done = 0;
  int32_t left_steps_done = 0;
  int32_t right_steps_done = 0;

  while (left_steps_done < left_profile.total_steps ||
         right_steps_done < right_profile.total_steps) {

    float elapsed_s = (micros() - start_us) / 1000000.0f;

    // Calculate expected distance for each wheel at this time
    float left_expected = integrateDistance(left_profile, elapsed_s);
    float right_expected = integrateDistance(right_profile, elapsed_s);

    // Step if behind
    bool did_step = false;
    if (left_steps_done < left_profile.total_steps &&
        left_distance_done < left_expected) {
      left_wheel.step();
      left_steps_done++;
      left_distance_done += step_distance_mm;
      did_step = true;
    }

    if (right_steps_done < right_profile.total_steps &&
        right_distance_done < right_expected) {
      right_wheel.step();
      right_steps_done++;
      right_distance_done += step_distance_mm;
      did_step = true;
    }

    if (!did_step) {
      delayMicroseconds(100);
    } else {
      delayMicroseconds(50); // Minimum pulse spacing
    }

    // Timeout protection
    if (elapsed_s > motion_time_s + 1.0f)
      break;
  }

  ESP_LOGD(TAG, "  Arc complete: L=%ld/%ld, R=%ld/%ld steps", left_steps_done,
           left_profile.total_steps, right_steps_done,
           right_profile.total_steps);

  // Execute final rotation if needed
  if (fabs(rotation_after_arc) > ANGLE_TOLERANCE_RAD && rotation_time_s > 0) {
    executeRotation(rotation_after_arc, rotation_time_s);
  }
}

// Helper to integrate distance traveled up to time t
float Robot::integrateDistance(const WheelMotion &m, float t) {
  if (t <= 0)
    return 0;
  if (t >= m.total_time_s)
    return m.distance_mm;

  float decel_start = m.accel_time_s + m.cruise_time_s;

  if (t <= m.accel_time_s) {
    // In acceleration: d = 0.5 * a * t^2
    return 0.5f * m.accel_mm_s2 * t * t;
  } else if (t <= decel_start) {
    // In cruise: d = accel_dist + v * (t - accel_time)
    float accel_dist = 0.5f * m.accel_mm_s2 * m.accel_time_s * m.accel_time_s;
    return accel_dist + m.max_velocity_mm_s * (t - m.accel_time_s);
  } else {
    // In deceleration
    float accel_dist = 0.5f * m.accel_mm_s2 * m.accel_time_s * m.accel_time_s;
    float cruise_dist = m.max_velocity_mm_s * m.cruise_time_s;
    float t_decel = t - decel_start;
    float decel_dist = m.max_velocity_mm_s * t_decel -
                       0.5f * m.accel_mm_s2 * t_decel * t_decel;
    return accel_dist + cruise_dist + decel_dist;
  }
}

void Robot::setMotorTestVelocity(float m0_velocity_rad_s,
                                 float m1_velocity_rad_s) {
  motor_test_active = true;
  target_m0_velocity_rad_s = m0_velocity_rad_s;
  target_m1_velocity_rad_s = m1_velocity_rad_s;

  g_motor_test_robot = this;
  left_wheel.enable();
  right_wheel.enable();

  float steps_per_rad = steps_per_revolution / (2.0f * M_PI);

  // M0 setup
  float m0_abs_vel = fabs(m0_velocity_rad_s);
  if (m0_abs_vel > 0.001f) {
    left_wheel.setDirection(m0_velocity_rad_s >= 0.0f);

    if (!left_wheel.configureTimer(m0_abs_vel, steps_per_rad, 0)) {
      ESP_LOGE(TAG, "M0 timer configure failed");
      return;
    }

    if (left_wheel.isTimerRunning()) {
      if (!left_wheel.stopTimer()) {
        ESP_LOGE(TAG, "M0 timer stop failed");
        return;
      }
    }
    if (!left_wheel.startTimer()) {
      ESP_LOGE(TAG, "M0 timer start failed");
      return;
    }
  } else if (left_wheel.isTimerRunning()) {
    if (!left_wheel.stopTimer()) {
      ESP_LOGE(TAG, "M0 timer stop failed");
      return;
    }
  }

  // M1 setup
  float m1_abs_vel = fabs(m1_velocity_rad_s);
  if (m1_abs_vel > 0.001f) {
    right_wheel.setDirection(m1_velocity_rad_s >= 0.0f);

    if (!right_wheel.configureTimer(m1_abs_vel, steps_per_rad, 0)) {
      ESP_LOGE(TAG, "M1 timer configure failed");
      return;
    }

    if (right_wheel.isTimerRunning()) {
      if (!right_wheel.stopTimer()) {
        ESP_LOGE(TAG, "M1 timer stop failed");
        return;
      }
    }

    if (!right_wheel.startTimer()) {
      ESP_LOGE(TAG, "M1 timer start failed");
      return;
    }
  } else if (right_wheel.isTimerRunning()) {
    if (!right_wheel.stopTimer()) {
      ESP_LOGE(TAG, "M1 timer stop failed");
      return;
    }
  }
}

void Robot::stopMotorTest() {
  motor_test_active = false;
  target_m0_velocity_rad_s = 0.0f;
  target_m1_velocity_rad_s = 0.0f;
  current_m0_velocity_rad_s = 0.0f;
  current_m1_velocity_rad_s = 0.0f;

  if (left_wheel.isTimerRunning()) {
    if (!left_wheel.stopTimer()) {
      ESP_LOGE(TAG, "M0 timer stop failed");
      return;
    }
  }
  if (right_wheel.isTimerRunning()) {
    if (!right_wheel.stopTimer()) {
      ESP_LOGE(TAG, "M1 timer stop failed");
      return;
    }
  }

  left_wheel.disable();
  right_wheel.disable();
}

void Robot::setBatteryVoltage(float voltage) { battery_voltage = voltage; }

void Robot::disableMotors() {
  left_wheel.disable();
  right_wheel.disable();
}

void Robot::enableMotors() {
  left_wheel.enable();
  right_wheel.enable();
}

void Robot::setStatus(uint8_t status) { system_status = status; }

uint8_t Robot::getStatus() { return system_status; }
