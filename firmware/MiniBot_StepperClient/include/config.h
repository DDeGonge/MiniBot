#ifndef __CONFIG_H__
#define __CONFIG_H__

// ============================================================================
// Logging and Debug Configuration
// ============================================================================
// Log levels: NONE=0, ERROR=1, WARN=2, INFO=3, DEBUG=4, VERBOSE=5
// Set to DEBUG (4) to enable verbose debug output for a component.
// Set to INFO (3) for normal operation.

#define LOG_LEVEL_MAIN 3
#define LOG_LEVEL_BATTERY 3
#define LOG_LEVEL_DEVICE_ID 3
#define LOG_LEVEL_ESPNOW 3
#define LOG_LEVEL_KINEMATICS 3
#define LOG_LEVEL_POS_EST 3
#define LOG_LEVEL_MMC5633 3
#define LOG_LEVEL_ROBOT 3

#define SPAM_POSITION false
#define ENABLE_BOT_WHILE_CHARGING true
#define CHARGE_TIMER_S 7200

// ============================================================================
// GPIO Pin Definitions
// ============================================================================

#define L_WHEEL_STEP_PIN 10
#define L_WHEEL_DIR_PIN 4
#define R_WHEEL_STEP_PIN 6
#define R_WHEEL_DIR_PIN 5
#define STEPPER_EN_PIN 7
#define STEPPER_RST_PIN 3

#define BATTERY_VOLTAGE_PIN 1
#define LED_PIN 0

#define SDA_PIN RX
#define SCL_PIN TX

// ============================================================================
// Battery Monitor
// ============================================================================

#define BATTERY_VOLTAGE_DIVIDER_RATIO 1.84f
#define BATTERY_CRITICAL_VOLTAGE 3.2f
#define BATTERY_CHARGING_VOLTAGE 4.6f
#define BATTERY_POLL_INTERVAL_MS 500
#define BATTERY_AVG_WINDOW_SIZE 10

// ============================================================================
// Robot Physical Configuration
// ============================================================================

// Wheel dimensions
#define WHEEL_RADIUS_MM 5.25f  // Wheel radius in millimeters
#define WHEEL_SPACING_MM 23.4f // Distance between wheel centers

// Stepper motor configuration
#define STEPS_PER_REVOLUTION                                                   \
  160.0f // Total microsteps per revolution. Motors are 20 full steps/rev

// Microstepping config
// STSPIN220 microstepping table (MS1=HIGH, MS2=LOW hardwired):
// STEP=0 DIR=0 -> 1/128, STEP=0 DIR=1 -> 1/256
// STEP=1 DIR=0 -> 1/2, STEP=1 DIR=1 -> 1/8
#define MSET_STEP_LVL true
#define MSET_DIR_LVL true

// Motor reversal
#define L_WHEEL_REVERSE true
#define R_WHEEL_REVERSE true

// ============================================================================
// Task Configuration
// ============================================================================

#define KINEMATICS_TASK_PRIORITY 5
#define POSITION_EST_SENSOR_PRIORITY 4
#define ESP_NOW_COMM_PRIORITY 3
#define POSITION_EST_CALC_PRIORITY 3
#define BATTERY_MONITOR_PRIORITY 2
#define LED_STATUS_PRIORITY 1

// ============================================================================
// Motion Control Limits
// ============================================================================

#define ROBOT_MAX_VELOCITY_MM_S 250.0f // Maximum linear velocity (mm/s)
#define ROBOT_MAX_ACCEL_MM_S2 500.0f   // Maximum linear acceleration (mm/s²)
#define MAX_ROT_VEL_RAD_S 20.0f         // Maximum angular velocity (rad/s)
#define MAX_ROT_ACCEL_RAD_S2 50.0f     // Maximum angular acceleration (rad/s²)

// Motor test command configuration
#define MOTOR_TEST_TIMEOUT_MS 1000 // Timeout for motor test commands

#define POSITION_TOLERANCE_MM 2.0f // Position error tolerance
#define ANGLE_TOLERANCE_RAD 0.05f  // ~3 degrees angle tolerance
#define MIN_ARC_RADIUS_MM 5.0f     // Minimum arc radius before fallback

// ============================================================================
// Motion Queue Configuration
// ============================================================================

#define MOTION_QUEUE_SIZE 8 // Maximum pending motion commands

// ============================================================================
// ESP-NOW Network Configuration
// ============================================================================

// Device ID functions are in device_id.h
#define WIFI_CHANNEL 6 // WiFi channel for ESP-NOW
#define WIFI_POWER WIFI_POWER_8_5dBm

// EXPERIMENTAL power saving config through duty cycling radio
#define ESPNOW_WAKE_WINDOW_MS 4    // Radio-on time per duty cycle
#define ESPNOW_WAKE_INTERVAL_MS 40 // Duty cycle period
#define SYNC_DUTY_CYCLE_TIMEOUT_MS                                             \
  30000 // Disable duty cycling if sync becomes stale

// ============================================================================
// Electromagnet Positioning System Configuration
// ============================================================================

// Electromagnet positions (x, y) in mm relative to the platform origin
// Add one entry per electromagnet; array length must match EMAG_COUNT
// TODO eventually these should be stored on the base station and sent
// to each robot over ESP-NOW

#define EMAG_POSITIONS_MM                                                      \
  {                                                                            \
      {40.0f, 138.79f}, /* EMAG 1 */                                           \
      {84.0f, 215.0f},  /* EMAG 2 */                                           \
      {40.0f, 291.21f}, /* EMAG 3 */                                           \
  }

// #define EMAG_POSITIONS_MM {                                                    \
//   {84.0f, 62.58f},    /* EMAG 0 */                                             \
//   {40.0f, 138.79f},   /* EMAG 1 */                                             \
//   {84.0f, 215.0f},    /* EMAG 2 */                                             \
//   {40.0f, 291.21f},   /* EMAG 3 */                                             \
//   {84.0f, 367.42f},   /* EMAG 4 */                                             \
//   {172.0f, 62.58f},   /* EMAG 5 */                                             \
//   {216.0f, 138.79f},  /* EMAG 6 */                                             \
//   {172.0f, 215.0f},   /* EMAG 7 */                                             \
//   {216.0f, 291.21f},  /* EMAG 8 */                                             \
//   {172.0f, 367.42f},  /* EMAG 9 */                                             \
//   {348.0f, 62.58f},   /* EMAG 10 */                                            \
//   {303.0f, 138.79f},  /* EMAG 11 */                                            \
//   {348.0f, 215.0f},   /* EMAG 12 */                                            \
//   {314.0f, 291.21f},  /* EMAG 13 */                                            \
//   {348.0f, 367.42f},  /* EMAG 14 */                                            \
//   {436.0f, 62.58f},   /* EMAG 15 */                                            \
//   {480.0f, 138.79f},  /* EMAG 16 */                                            \
//   {436.0f, 215.0f},   /* EMAG 17 */                                            \
//   {480.0f, 291.21f},  /* EMAG 18 */                                            \
//   {436.0f, 367.42f},  /* EMAG 19 */                                            \
// }

// Electromagnet frame timing setup
#define EMAG_FRAME_LEN_MS 200 // Total frame length
#define EMAG_COUNT 3          // Number of electromagnets in platform
#define EMAG_FWD_ON_TIME_MS 7 // How long forward power is applied
#define EMAG_REV_ON_TIME_MS 7 // How long reverse power is applied
#define EMAG_TRIM_MS                                                           \
  1.1 // Samples closer than this to state changes are ignored
static_assert(EMAG_COUNT * (EMAG_FWD_ON_TIME_MS + EMAG_REV_ON_TIME_MS) <=
                  EMAG_FRAME_LEN_MS,
              "EMAG slot time * EMAG_COUNT exceeds EMAG_FRAME_LEN_MS");

// Sampling
#define EMAG_MIN_SAMPLE_PERIOD_US 950 // 1 kHz sampling rate but actually 1050hz measured
#define EMAG_SAMPLE_TIME_US 1200 // Sensor read delay, needed for time sync
#define MAX_SAMPLES_PER_EMAG                                                   \
  ((EMAG_FWD_ON_TIME_MS + EMAG_REV_ON_TIME_MS) * 1000 /                        \
   EMAG_MIN_SAMPLE_PERIOD_US)

// Detection thresholds
#define FIELD_THRESHOLD_GAUSS 0.5f // Minimum field magnitude to consider valid

// Magnetometer sensor offset from robot center, in robot body frame (mm)
// Positive X = forward, positive Y = left
#define SENSOR_OFFSET_X_MM -7.3f
#define SENSOR_OFFSET_Y_MM 8.1f

// Position estimation parameters
#define TRUE_POSE_LPF_CUTOFF_HZ 1.0f
#define TRUE_POSE_LPF_REF_CONFIDENCE                                           \
  2.0f // Ref confidence for full LPF response; lower=faster tracking
#define TRUE_POSE_STALE_TIMEOUT_MS                                             \
  1000 // Max age of true pose before considered stale
#define EMAG_MIN_SIGNAL_GAUSS                                                  \
  0.5f // Minimum fwd-rev differential magnitude to consider reading
       // valid
#define EMAG_MAX_ANGLE_DELTA_RAD                                               \
  0.6f // Maximum allowed difference between forward and reverse azimuth angles
       // for the same emag

#endif // __CONFIG_H__
