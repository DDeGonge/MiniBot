#ifndef __KINEMATICS_CONTROLLER_H__
#define __KINEMATICS_CONTROLLER_H__

#include "motion_queue.h"
#include "motor_test_queue.h"
#include "robot.h"

/**
 * Kinematics Controller Task
 *
 * Priority: HIGHEST
 * Responsible for:
 * - Consuming motion commands from the motion queue
 * - Consuming motor test commands from the motor test queue
 * - Computing inverse kinematics to convert Cartesian targets to motor commands
 * - Controlling stepper motors to reach target positions
 * - Executing smooth motion trajectories
 * - Updating motor test velocities with ramping
 *
 * @param pvParameters Pointer to initialization parameters (unused)
 */
void KinematicsController_Task(void *pvParameters);

/**
 * Initialize the kinematics controller
 * Should be called before starting the task
 *
 * @param motion_queue MotionQueue handle
 * @param motor_test_queue MotorTestQueue handle
 * @return true on success, false on failure
 */
bool KinematicsController_Init(MotionQueue motion_queue,
                               MotorTestQueue motor_test_q);

#endif // __KINEMATICS_CONTROLLER_H__
