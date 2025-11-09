/**
 * @file actions.h
 * @brief High-level robot actions - SIMPLIFIED (Attacker/Defender only)
 * @author Agustin Valenzuela, Alex Petersen, Dylan Frigerio, Enzo Fernandez Rosas
 * @copyright Copyright (c) 2025
 */

#ifndef ACTIONS_H
#define ACTIONS_H

#include "robot.h"
#include "strategy.h"

struct ActionResult {
    bool success;
    const char* description;
};

// Ball control actions
namespace BallControl {
    void interceptBall(const RobotState& robot, const RobotState& ball,
        RobotCommand& cmd);

    void chaseBall(const RobotState& robot, const RobotState& ball,
        RobotCommand& cmd);

    void dribbleTo(const RobotState& robot, const RobotState& ball,
        float targetX, float targetZ, RobotCommand& cmd);

    /**
     * Dribble toward goal with slow rotation for better control
     */
    void dribbleToGoal(const RobotState& robot, const RobotState& ball,
        RobotCommand& cmd);
}

// Shooting actions
namespace Shooting {
    ActionResult shootAtGoal(const RobotState& robot, const RobotState& ball,
        RobotCommand& cmd);

    void clearBall(const RobotState& robot, const RobotState& ball,
        RobotCommand& cmd);
}

// Positioning actions
namespace Positioning {
    void holdPosition(const RobotState& robot, RobotCommand& cmd);

    void moveTo(const RobotState& robot, float targetX, float targetZ,
        RobotCommand& cmd);

    void faceTowards(const RobotState& robot, float pointX, float pointZ,
        RobotCommand& cmd);

    /**
     * Move to position while facing a specific point
     */
    void moveToWhileFacing(const RobotState& robot,
        float targetX, float targetZ,
        float faceX, float faceZ,
        RobotCommand& cmd);
}

// Utility functions
bool hasBallControl(const RobotState& robot, const RobotState& ball);
bool canKickBall(const RobotState& robot, const RobotState& ball);

/**
 * Check if position is inside a goal area
 */
bool isInsideGoalArea(float x, float z);

/**
 * Clamp position to valid field boundaries (avoiding goal areas and edges)
 */
void clampToFieldBounds(float& x, float& z, bool avoidGoalAreas = true);

#endif // ACTIONS_H