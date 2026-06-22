#ifndef __ROBOT_H__
#define __ROBOT_H__

#include "config.h"
#include "esp_now_communicator.h"
#include "motion_queue.h"
#include "stepper.h"
#include <Arduino.h>
#include <driver/timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <functional>
#include <stdbool.h>
#include <stdint.h>

enum class MotionType { NONE, ROTATION_ONLY, STRAIGHT_LINE, ARC_THEN_ROTATE };

class Robot {
public:
  // Motion profile calculation struct
  struct MotionProfile {
    float accel_time_s;
    float accel_phase_ms;
    bool full_profile;
  };

  // Wheel motion profile for individual wheel control
  struct WheelMotion {
    float distance_mm;
    float max_velocity_mm_s;
    float accel_mm_s2;
    int32_t total_steps;
    bool forward;
    float total_time_s;
    float accel_time_s;
    float cruise_time_s;
    bool is_triangular;
  };

private:
  // Robot config
  float wheel_radius_mm;
  float wheel_spacing_mm;

  // Robot pose (active/commanded position)
  float positionX;
  float positionY;
  float orientation;

  // True position estimate (continuously updated by position estimator)
  float true_x;
  float true_y;
  float true_theta;
  bool true_pose_initialized = false;
  uint32_t true_pose_last_update_us = 0;
  SemaphoreHandle_t true_pose_mutex = NULL;

  float battery_voltage;
  float low_battery_v_threshold = BATTERY_CRITICAL_VOLTAGE;
  uint8_t system_status;

  // Motion state tracking
  bool is_moving;

  // Motor test state tracking
  bool motor_test_active = false;
  float target_m0_velocity_rad_s = 0.0f;
  float target_m1_velocity_rad_s = 0.0f;
  float current_m0_velocity_rad_s = 0.0f;
  float current_m1_velocity_rad_s = 0.0f;
  uint32_t motor_test_last_update_time = 0;

  // Motor and motion control constants - initialized from config.h but
  // modifiable
  float steps_per_revolution;
  float robot_max_velocity_mm_s;
  float robot_max_accel_mm_s2;
  float max_rot_vel_rad_s;
  float max_rot_accel_rad_s2;
  float stepper_max_velocity_mm_s;

public:
  StepperDriver left_wheel;
  StepperDriver right_wheel;

  /**
   * Initialize the robot with default hardware configuration
   * @return true on success, false on failure
   */
  bool initialize();

  /**
   * Update the true position estimate based on wheel step counts
   * Uses odometry to calculate position change
   * This is called continuously by the position estimator task
   */
  void updateEstimatedPosition();

  /**
   * Set the true position estimate from external source (e.g., magnetic
   * positioning)
   * @param x X position in mm
   * @param y Y position in mm
   * @param theta Orientation in radians
   * @param confidence Solution confidence (higher = more responsive LPF)
   */
  void setTruePose(float x, float y, float theta, float confidence = 1.0f);

  /**
   * Get current robot sensed position
   * @param x Pointer to store X position
   * @param y Pointer to store Y position
   * @param orientation Pointer to store orientation
   */
  /**
   * Get the true position estimate.
   * @return false if data has never been set or has not been updated within
   * TRUE_POSE_STALE_TIMEOUT_MS
   */
  bool getTruePose(float *x, float *y, float *orientation);

  /**
   * Copy the true position estimate to the active position
   * This should be called by tasks that need to sync the active position
   * with the latest position estimate (e.g., after motion completion)
   */
  void updatePositionFromTruePose();

  /**
   * Get current robot position
   * @param x Pointer to store X position
   * @param y Pointer to store Y position
   * @param orientation Pointer to store orientation
   */
  void getPosition(float *x, float *y, float *orientation);

  /**
   * Set robot position (for external updates, e.g., from position estimator)
   * @param x X position
   * @param y Y position
   * @param orientation Robot orientation
   */
  void setPosition(float x, float y, float orientation);

  /**
   * Set target position and orientation, always absolute coordinates
   * @param target MotionCommand struct with target pose
   */
  void setTargetPose(MotionCommand target);

  /**
   * Set motor test velocities and start motor test mode
   * @param m0_velocity_rad_s Motor 0 target velocity in rad/s
   * @param m1_velocity_rad_s Motor 1 target velocity in rad/s
   */
  void setMotorTestVelocity(float m0_velocity_rad_s, float m1_velocity_rad_s);

  /**
   * Stop motor test mode
   */
  void stopMotorTest();

  /**
   * Update motor test stepping with time delta
   * Called periodically by kinematics controller to update motor velocities
   * with ramping and execute stepping
   * @param dt_ms Time delta since last update in milliseconds
   */
  void updateMotorTest(uint32_t dt_ms);

  /**
   * Set battery voltage (from battery monitor task)
   * @param voltage Battery voltage in volts
   */
  void setBatteryVoltage(float voltage);

  /**
   * Get battery voltage
   * @return Battery voltage in volts
   */
  float getBatteryVoltage() const { return battery_voltage; }

  /**
   * Get battery voltage
   * @return Battery voltage in volts
   */
  bool getBatteryCritical() const {
    return battery_voltage < low_battery_v_threshold;
  }

  /**
   * Get battery voltage
   * @return Battery voltage in volts
   */
  bool getBatteryCharging() const {
    return battery_voltage >= BATTERY_CHARGING_VOLTAGE;
  }

  /**
   * Disable both stepper motor drivers
   */
  void disableMotors();

  /**
   * Enable both stepper motor drivers
   */
  void enableMotors();

  /**
   * Check if robot is currently executing a motion
   * @return true if moving, false if idle
   */
  bool isMoving() const { return is_moving; }

  // Motion control configuration getters
  float getStepsPerRevolution() const { return steps_per_revolution; }
  float getRobotMaxVelocity() const { return robot_max_velocity_mm_s; }
  float getRobotMaxAccel() const { return robot_max_accel_mm_s2; }
  float getMaxRotVel() const { return max_rot_vel_rad_s; }
  float getMaxRotAccel() const { return max_rot_accel_rad_s2; }
  float getStepperMaxVelocity() const { return stepper_max_velocity_mm_s; }

  // Motion control configuration setters
  void setStepsPerRevolution(float steps) { steps_per_revolution = steps; }
  void setRobotMaxVelocity(float vel) { robot_max_velocity_mm_s = vel; }
  void setRobotMaxAccel(float accel) { robot_max_accel_mm_s2 = accel; }
  void setMaxRotVel(float vel) { max_rot_vel_rad_s = vel; }
  void setMaxRotAccel(float accel) { max_rot_accel_rad_s2 = accel; }
  void setStepperMaxVelocity(float vel) { stepper_max_velocity_mm_s = vel; }

private:
  // Legacy motion profile (still used by executeMotionLoop)
  MotionProfile calculateMotionProfile(float distance_mm,
                                       float max_velocity_mm_s);

  float calculateVelocityAtTime(float elapsed_ms, float remaining_ms,
                                float accel_phase_ms, float max_velocity_mm_s,
                                bool full_profile);

  float calculateMaxVelocityForTime(float distance_mm, float target_time_s,
                                    float max_accel);

  void executeMotionLoop(int32_t total_steps, float base_step_time_us,
                         float max_velocity_mm_s, const MotionProfile &profile,
                         TickType_t start_tick, TickType_t target_end_tick,
                         std::function<void(int32_t)> step_callback);

  // Shared accel/cruise/timer/decel step loop for synchronized wheel pairs
  void executeStepLoop(const WheelMotion &profile, float steps_per_mm);

  // New motion execution methods
  void executeRotation(float angle_rad, float target_time_s);
  void executeStraightLine(float distance_mm, float target_time_s);
  void executeArcToPosition(float dx, float dy, float current_theta,
                            float target_time_s, float final_angle_delta);

  // Helper to integrate wheel distance from motion profile
  float integrateDistance(const WheelMotion &m, float t);

public:
  /**
   * Set system status
   * @param status Status code
   */
  void setStatus(uint8_t status);

  /**
   * Get system status
   * @return Current system status code
   */
  uint8_t getStatus();
};

#endif // __ROBOT_H__