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

    // Constructor with default values
    RobotState() : posX(0), posY(0), posZ(0),
        rotX(0), rotY(0), rotZ(0),
        velX(0), velY(0), velZ(0),
        angVelX(0), angVelY(0), angVelZ(0) {
    }
};

// Robot control commands
struct RobotCommand {
    float targetX, targetZ;      // Target position
    float targetRotY;            // Target rotation
    float dribbler;              // Dribbler power [0-1]
    float kick;                  // Kick power [0-1]
    float chip;                  // Chip power [0-1]

    // Constructor with default values (no action)
    RobotCommand() : targetX(0), targetZ(0), targetRotY(0),
        dribbler(0), kick(0), chip(0) {
    }

    // Reset command to neutral state
    void reset() {
        dribbler = 0.0f;
        kick = 0.0f;
        chip = 0.0f;
    }
};

#endif // ROBOT_H