/**
 * @file robot.h
 * @brief Robot state and command structures
 * @author Agustin Valenzuela,
 *         Alex Petersen,
 *         Dylan Frigerio,
 *         Enzo Fernandez Rosas
 *
 * @copyright Copyright (c) 2025
 */

#ifndef ROBOT_H
#define ROBOT_H

 // Robot state information
struct RobotState {
    float posX, posY, posZ;      // Position
    float rotX, rotY, rotZ;      // Rotation
    float velX, velY, velZ;      // Velocity
    float angVelX, angVelY, angVelZ; // Angular velocity
};

// Robot control commands
struct RobotCommand {
    float targetX, targetZ;      // Target position
    float targetRotY;            // Target rotation
    float dribbler;              // Dribbler power [0-1]
    float kick;                  // Kick power [0-1]
    float chip;                  // Chip power [0-1]
};

#endif // ROBOT_H