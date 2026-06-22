#ifndef __MESSAGES_IPC_H__
#define __MESSAGES_IPC_H__

/**
 * 2D position of a single electromagnet on the platform (mm)
 */
typedef struct {
  float x;
  float y;
} EmagPosition;

/**
 * Motion command structure passed from communicator to kinematics controller
 */
typedef struct {
  float target_position_x_mm;
  float target_position_y_mm;
  float target_orientation_rad;
  float move_duration_ms;
} MotionCommand;

/**
 * Motor test command structure passed from communicator to kinematics
 * controller
 */
typedef struct {
  float m0_velocity_rad_s; // Motor 0 target velocity in rad/s
  float m1_velocity_rad_s; // Motor 1 target velocity in rad/s
} MotorTestRequest;

#endif // __MESSAGES_IPC_H__