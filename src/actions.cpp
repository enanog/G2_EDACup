/**
 * @file actions.cpp
 * @brief Implementation of robot actions
 * @author Agustin Valenzuela,
 *         Alex Petersen,
 *         Dylan Frigerio,
 *         Enzo Fernandez Rosas
 *
 * @copyright Copyright (c) 2025
 */

#include "actions.h"
#include "geometry.h"
#include "constants.h"
#include <cmath>
#include <algorithm>
#include <iostream>

 // Smooth rotation speed (radians per frame)
const float SMOOTH_ROTATION_SPEED = 0.15f;

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

/**
 * Check if robot controls the ball
 */
bool hasBallControl(const RobotState& robot, const RobotState& ball) {
    float dist = distance(robot.posX, robot.posZ, ball.posX, ball.posZ);
    return dist < BALL_CONTROL_DISTANCE;
}

/**
 * Check if robot can kick the ball
 * Must be close and facing the ball
 */
bool canKickBall(const RobotState& robot, const RobotState& ball) {
    float dist = distance(robot.posX, robot.posZ, ball.posX, ball.posZ);
    if (dist > BALL_CONTROL_DISTANCE) return false;

    // Check if robot is facing the ball
    float angleToBall = angleTo(robot.posX, robot.posZ, ball.posX, ball.posZ);
    float angleDiff = std::abs(angleDifference(robot.rotY, angleToBall));

    std::cerr << "  Can kick check: dist=" << dist
        << ", angleDiff=" << (angleDiff * 180.0f / M_PI) << "°" << std::endl;

    // Within 30 degrees of facing ball
    return (angleDiff < 30.0f * DEG_TO_RAD);
}

// ============================================================================
// BALL CONTROL ACTIONS
// ============================================================================

/**
 * Intercept ball - move to ball with dribbler active
 */
void BallControl::interceptBall(const RobotState& robot, const RobotState& ball,
    RobotCommand& cmd) {
    cmd.targetX = ball.posX;
    cmd.targetZ = ball.posZ;
    cmd.targetRotY = smoothRotation(robot.rotY,
        angleTo(robot.posX, robot.posZ, ball.posX, ball.posZ),
        SMOOTH_ROTATION_SPEED);
    cmd.dribbler = 0.8f;
    cmd.kick = 0.0f;
    cmd.chip = 0.0f;

    std::cerr << "  Action: Intercepting ball" << std::endl;
}

/**
 * Chase ball - fast approach
 */
void BallControl::chaseBall(const RobotState& robot, const RobotState& ball,
    RobotCommand& cmd) {
    cmd.targetX = ball.posX;
    cmd.targetZ = ball.posZ;
    cmd.targetRotY = smoothRotation(robot.rotY,
        angleTo(robot.posX, robot.posZ, ball.posX, ball.posZ),
        SMOOTH_ROTATION_SPEED);
    cmd.dribbler = 0.9f;
    cmd.kick = 0.0f;
    cmd.chip = 0.0f;

    std::cerr << "  Action: Chasing ball" << std::endl;
}

/**
 * Dribble ball toward target
 */
void BallControl::dribbleTo(const RobotState& robot, const RobotState& ball,
    float targetX, float targetZ, RobotCommand& cmd) {
    // Calculate direction to target
    float dirX = targetX - robot.posX;
    float dirZ = targetZ - robot.posZ;
    float len = std::sqrt(dirX * dirX + dirZ * dirZ);

    if (len > 0.01f) {
        dirX /= len;
        dirZ /= len;
    }

    // Move in small steps toward target
    const float STEP = 0.05f;
    cmd.targetX = robot.posX + dirX * STEP;
    cmd.targetZ = robot.posZ + dirZ * STEP;

    // Face toward target
    float targetAngle = angleTo(robot.posX, robot.posZ, targetX, targetZ);
    cmd.targetRotY = smoothRotation(robot.rotY, targetAngle, SMOOTH_ROTATION_SPEED);

    cmd.dribbler = 1.0f;
    cmd.kick = 0.0f;
    cmd.chip = 0.0f;

    std::cerr << "  Action: Dribbling to (" << targetX << ", " << targetZ << ")" << std::endl;
}

// ============================================================================
// SHOOTING ACTIONS
// ============================================================================

/**
 * Shoot at goal
 */
ActionResult Shooting::shootAtGoal(const RobotState& robot, const RobotState& ball,
    RobotCommand& cmd) {
    ActionResult result;
    result.success = false;

    // Target center of opponent goal
    float targetX = RIGHT_GOAL_X;
    float targetZ = 0.0f;

    // Calculate angle to goal
    float angleToGoal = angleTo(ball.posX, ball.posZ, targetX, targetZ);

    // Check if we can kick
    if (!canKickBall(robot, ball)) {
        // Position behind ball for shooting
        float offset = 0.15f;
        Vec2 goalDir = getFacingVector(angleToGoal);

        // Move to position behind ball (opposite to goal direction)
        cmd.targetX = ball.posX - goalDir.x * offset;
        cmd.targetZ = ball.posZ - goalDir.z * offset;
        cmd.targetRotY = smoothRotation(robot.rotY, angleToGoal, SMOOTH_ROTATION_SPEED);
        cmd.dribbler = 0.8f;
        cmd.kick = 0.0f;
        cmd.chip = 0.0f;

        result.description = "Positioning for shot";
        std::cerr << "  Action: Positioning for shot" << std::endl;
        return result;
    }

    // Check if aligned with goal
    float angleDiff = std::abs(angleDifference(robot.rotY, angleToGoal));

    if (angleDiff > 15.0f * DEG_TO_RAD) {
        // Need better alignment
        cmd.targetX = robot.posX;
        cmd.targetZ = robot.posZ;
        cmd.targetRotY = smoothRotation(robot.rotY, angleToGoal, SMOOTH_ROTATION_SPEED);
        cmd.dribbler = 0.8f;
        cmd.kick = 0.0f;
        cmd.chip = 0.0f;

        result.description = "Aligning for shot";
        std::cerr << "  Action: Aligning for shot (diff: "
            << (angleDiff * 180.0f / M_PI) << "°)" << std::endl;
        return result;
    }

    // Execute shot
    float distToGoal = distance(ball.posX, ball.posZ, targetX, targetZ);
    float power = std::clamp(distToGoal / 2.0f, 0.6f, 1.0f);

    result.success = true;
    cmd.targetX = ball.posX;
    cmd.targetZ = ball.posZ;
    cmd.targetRotY = angleToGoal;
    cmd.dribbler = 0.0f;
    cmd.kick = power;
    cmd.chip = 0.0f;
    result.description = "Shooting at goal";

    std::cerr << "  Action: ⚽ SHOOTING! Power: " << power
        << ", Angle: " << (angleToGoal * 180.0f / M_PI) << "°" << std::endl;

    return result;
}

/**
 * Clear ball away from goal
 */
void Shooting::clearBall(const RobotState& robot, const RobotState& ball,
    RobotCommand& cmd) {
    // Kick ball toward opponent goal
    float targetX = RIGHT_GOAL_X;
    float targetZ = (ball.posZ > 0) ? -FIELD_HALF_WIDTH : FIELD_HALF_WIDTH;

    float angleToTarget = angleTo(ball.posX, ball.posZ, targetX, targetZ);

    if (canKickBall(robot, ball)) {
        cmd.targetX = ball.posX;
        cmd.targetZ = ball.posZ;
        cmd.targetRotY = angleToTarget;
        cmd.dribbler = 0.0f;
        cmd.kick = 1.0f;
        cmd.chip = 0.0f;

        std::cerr << "  Action: CLEARING ball!" << std::endl;
    }
    else {
        // Position behind ball for clearing
        float offset = 0.15f;
        Vec2 clearDir = getFacingVector(angleToTarget);

        cmd.targetX = ball.posX - clearDir.x * offset;
        cmd.targetZ = ball.posZ - clearDir.z * offset;
        cmd.targetRotY = smoothRotation(robot.rotY, angleToTarget, SMOOTH_ROTATION_SPEED);
        cmd.dribbler = 0.5f;
        cmd.kick = 0.0f;
        cmd.chip = 0.0f;

        std::cerr << "  Action: Positioning to clear ball" << std::endl;
    }
}

// ============================================================================
// POSITIONING ACTIONS
// ============================================================================

/**
 * Hold current position
 */
void Positioning::holdPosition(const RobotState& robot, RobotCommand& cmd) {
    cmd.targetX = robot.posX;
    cmd.targetZ = robot.posZ;
    cmd.targetRotY = robot.rotY;
    cmd.dribbler = 0.0f;
    cmd.kick = 0.0f;
    cmd.chip = 0.0f;
}

/**
 * Move to target position
 */
void Positioning::moveTo(const RobotState& robot, float targetX, float targetZ,
    RobotCommand& cmd) {
    cmd.targetX = targetX;
    cmd.targetZ = targetZ;

    // Face direction of movement
    cmd.targetRotY = smoothRotation(robot.rotY,
        angleTo(robot.posX, robot.posZ, targetX, targetZ),
        SMOOTH_ROTATION_SPEED);
    cmd.dribbler = 0.0f;
    cmd.kick = 0.0f;
    cmd.chip = 0.0f;
}

/**
 * Face toward a point
 */
void Positioning::faceTowards(const RobotState& robot, float pointX, float pointZ,
    RobotCommand& cmd) {
    cmd.targetX = robot.posX;
    cmd.targetZ = robot.posZ;
    cmd.targetRotY = smoothRotation(robot.rotY,
        angleTo(robot.posX, robot.posZ, pointX, pointZ),
        SMOOTH_ROTATION_SPEED);
    cmd.dribbler = 0.0f;
    cmd.kick = 0.0f;
    cmd.chip = 0.0f;
}