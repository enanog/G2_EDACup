/**
 * @file robot.cpp
 * @brief Implementation of robot control functions
 * @author Agustin Valenzuela,
 *         Alex Petersen,
 *         Dylan Frigerio,
 *         Enzo Fernandez Rosas
 *
 * @copyright Copyright (c) 2025
 */

#include "robot.h"
#include "geometry.h"
#include "constants.h"
#include <cmath>
#include <iostream>

 // ============================================================================
 // BASIC CONTROL FUNCTIONS
 // ============================================================================

RobotCommand moveToPosition(const RobotState& robot, float targetX, float targetZ)
{
	RobotCommand cmd;

    cmd.targetX = targetX;
    cmd.targetZ = targetZ;

    // Face the target point
    cmd.targetRotY = angleTo(robot.posX, robot.posZ, targetX, targetZ);
   
    // No dribbler, kick or chip yet
    cmd.dribbler = 0.0f;
    cmd.kick = 0.0f;
    cmd.chip = 0.0f;

	/*std::cerr << "Moving to (" << targetX << ", " << targetZ << ") with angle " << cmd.targetRotY << std::endl;
	std::cerr << "Current position: (" << robot.posX << ", " << robot.posZ << ")" << std::endl;*/

	return cmd;
}