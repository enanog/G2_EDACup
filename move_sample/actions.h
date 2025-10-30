/**
 * @file actions.h
 * @brief High-level robot actions
 * @author Agustin Valenzuela,
 *         Alex Petersen,
 *         Dylan Frigerio,
 *         Enzo Fernandez Rosas
 *
 * @copyright Copyright (c) 2025
 */

#ifndef ACTIONS_H
#define ACTIONS_H

#include "robot.h"
#include "strategy.h"

 // Action result with success status
struct ActionResult {
    bool success;
    RobotCommand command;
    const char* description;
};

// Ball control actions
namespace BallControl {
    RobotCommand interceptBall(const RobotState& robot, const RobotState& ball);
    RobotCommand chaseBall(const RobotState& robot, const RobotState& ball);
    RobotCommand dribbleTo(const RobotState& robot, const RobotState& ball,
        float targetX, float targetZ);
}

// Shooting actions
namespace Shooting {
    ActionResult shootAtGoal(const RobotState& robot, const RobotState& ball);
    RobotCommand clearBall(const RobotState& robot, const RobotState& ball);
}

// Positioning actions
namespace Positioning {
    RobotCommand moveTo(const RobotState& robot, float targetX, float targetZ);
}

// Utility functions
bool hasBallControl(const RobotState& robot, const RobotState& ball);
bool canKickBall(const RobotState& robot, const RobotState& ball);

#endif // ACTIONS_H