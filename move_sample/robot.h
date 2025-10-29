/**
 * @file robot.h
 * @brief Robot state representation and control functions
 * @author Agustin Valenzuela,
 *         Alex Petersen,
 *         Dylan Frigerio,
 *         Enzo Fernandez Rosas
 *
 * @copyright Copyright (c) 2025
 */

#ifndef ROBOT_H
#define ROBOT_H

 // ============================================================================
 // DATA STRUCTURES
 // ============================================================================

 /**
  * @brief Represents the complete state of a robot
  */
struct RobotState {
	float posX, posY, posZ;              // Position coordinates
	float rotX, rotY, rotZ;              // Rotation (Euler angles in radians)
	float velX, velY, velZ;              // Linear velocity
	float angVelX, angVelY, angVelZ;     // Angular velocity
};

/**
 * @brief Represents commands to be sent to a robot
 */
struct RobotCommand {
	float targetX, targetZ;    // Target position
	float targetRotY;          // Target rotation (Y axis)
	float dribbler;            // Dribbler speed [0-1]
	float kick;                // Kick power [0-1]
	float chip;                // Chip power [0-1]
};

/**
 * @brief Generate command to move robot to a specific position
 * @param robot Current robot state
 * @param targetX Target X coordinate
 * @param targetZ Target Z coordinate
 * @return Command to move to target position
 */
RobotCommand moveToPosition(const RobotState& robot, float targetX, float targetZ);

#endif // ROBOT_H