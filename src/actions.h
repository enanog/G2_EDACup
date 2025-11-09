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

 // Action result with success status and description
struct ActionResult {
    bool success;
    const char* description;
};

// Ball control actions - modify command directly
namespace BallControl {
    /**
     * Intercept ball - move to ball position with dribbler
     */
    void interceptBall(const RobotState& robot, const RobotState& ball,
        RobotCommand& cmd);

    /**
     * Chase ball - fast approach to ball
     */
    void chaseBall(const RobotState& robot, const RobotState& ball,
        RobotCommand& cmd);

    /**
     * Dribble ball toward target position
     */
    void dribbleTo(const RobotState& robot, const RobotState& ball,
        float targetX, float targetZ, RobotCommand& cmd);
}

// Shooting actions - modify command directly
namespace Shooting {
    /**
     * Shoot at goal - returns success status
     */
    ActionResult shootAtGoal(const RobotState& robot, const RobotState& ball,
        RobotCommand& cmd);

    /**
     * Clear ball away from goal
     */
    void clearBall(const RobotState& robot, const RobotState& ball,
        RobotCommand& cmd);
}

// Positioning actions - modify command directly
namespace Positioning {
    /**
     * Hold current position
     */
    void holdPosition(const RobotState& robot, RobotCommand& cmd);

    /**
     * Move to target position
     */
    void moveTo(const RobotState& robot, float targetX, float targetZ,
        RobotCommand& cmd);

    /**
     * Face toward a point
     */
    void faceTowards(const RobotState& robot, float pointX, float pointZ,
        RobotCommand& cmd);
}

// Utility functions
/**
 * Check if robot has control of the ball
 */
bool hasBallControl(const RobotState& robot, const RobotState& ball);

/**
 * Check if robot is in position to kick
 */
bool canKickBall(const RobotState& robot, const RobotState& ball);

#endif // ACTIONS_H