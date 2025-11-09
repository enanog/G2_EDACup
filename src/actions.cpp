/**
 * @file actions.cpp
 * @brief Implementation of robot actions - IMPROVED
 * @author Agustin Valenzuela, Alex Petersen, Dylan Frigerio, Enzo Fernandez Rosas
 * @copyright Copyright (c) 2025
 */

#include "actions.h"
#include "geometry.h"
#include "constants.h"
#include <cmath>
#include <algorithm>
#include <iostream>

 // Smooth rotation speed (radians per frame) - REDUCED for better control
const float SMOOTH_ROTATION_SPEED = 0.05f;
const float DRIBBLE_ROTATION_SPEED = 0.04f;  // Even slower when dribbling

// Movement speed control - REDUCED
const float MOVEMENT_STEP = 0.05f;  // Slower movement
const float DRIBBLE_STEP = 0.04f;   // Even slower when dribbling

// Field boundaries with safety margin
const float FIELD_X_MIN = -FIELD_HALF_LENGTH + 0.15f;
const float FIELD_X_MAX = FIELD_HALF_LENGTH - 0.15f;
const float FIELD_Z_MIN = -FIELD_HALF_WIDTH + 0.15f;
const float FIELD_Z_MAX = FIELD_HALF_WIDTH - 0.15f;

// Goal area boundaries (robots cannot enter)
const float OWN_AREA_X_MAX = LEFT_GOAL_X + 0.30f;      // Own goal area
const float OPPONENT_AREA_X_MIN = RIGHT_GOAL_X - 0.30f; // Opponent goal area
const float AREA_Z_HALF_WIDTH = 0.40f;  // Half width of goal area

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

/**
 * Check if position is inside a goal area
 */
bool isInsideGoalArea(float x, float z) {
    // Own goal area
    if (x < OWN_AREA_X_MAX && std::abs(z) < AREA_Z_HALF_WIDTH) {
        return true;
    }
    // Opponent goal area
    if (x > OPPONENT_AREA_X_MIN && std::abs(z) < AREA_Z_HALF_WIDTH) {
        return true;
    }
    return false;
}

/**
 * Clamp position to valid field boundaries (avoiding goal areas and edges)
 */
void clampToFieldBounds(float& x, float& z, bool avoidGoalAreas) {
    // Clamp to field edges
    x = std::clamp(x, FIELD_X_MIN, FIELD_X_MAX);
    z = std::clamp(z, FIELD_Z_MIN, FIELD_Z_MAX);

    if (avoidGoalAreas) {
        // Push out of own goal area
        if (x < OWN_AREA_X_MAX && std::abs(z) < AREA_Z_HALF_WIDTH) {
            x = OWN_AREA_X_MAX + 0.05f;  // Push forward out of area
        }

        // Push out of opponent goal area
        if (x > OPPONENT_AREA_X_MIN && std::abs(z) < AREA_Z_HALF_WIDTH) {
            x = OPPONENT_AREA_X_MIN - 0.05f;  // Push back out of area
        }
    }
}

bool hasBallControl(const RobotState& robot, const RobotState& ball) {
    float dist = distance(robot.posX, robot.posZ, ball.posX, ball.posZ);
    return dist < BALL_CONTROL_DISTANCE;
}

bool canKickBall(const RobotState& robot, const RobotState& ball) {
    float dist = distance(robot.posX, robot.posZ, ball.posX, ball.posZ);
    if (dist > BALL_CONTROL_DISTANCE) return false;

    float angleToBall = angleTo(robot.posX, robot.posZ, ball.posX, ball.posZ);
    float angleDiff = std::abs(angleDifference(robot.rotY, angleToBall));

    return (angleDiff < 30.0f * DEG_TO_RAD);
}

/**
 * Check if there's a clear path between two points (no opponents blocking)
 */
bool hasClearPath(float fromX, float fromZ, float toX, float toZ,
    const RobotState& opp1, const RobotState& opp2) {
    // Simplified: check if opponents are not too close to the line
    float lineLen = distance(fromX, fromZ, toX, toZ);
    if (lineLen < 0.01f) return true;

    // Check distance from opponents to the line
    Vec2 lineDir((toX - fromX) / lineLen, (toZ - fromZ) / lineLen);

    auto distToLine = [&](const RobotState& opp) {
        float dx = opp.posX - fromX;
        float dz = opp.posZ - fromZ;
        float proj = dx * lineDir.x + dz * lineDir.z;
        proj = std::clamp(proj, 0.0f, lineLen);
        float nearX = fromX + lineDir.x * proj;
        float nearZ = fromZ + lineDir.z * proj;
        return distance(nearX, nearZ, opp.posX, opp.posZ);
        };

    float dist1 = distToLine(opp1);
    float dist2 = distToLine(opp2);

    const float BLOCKING_DISTANCE = 0.25f;
    return (dist1 > BLOCKING_DISTANCE && dist2 > BLOCKING_DISTANCE);
}

// ============================================================================
// BALL CONTROL ACTIONS
// ============================================================================

void BallControl::interceptBall(const RobotState& robot, const RobotState& ball,
    RobotCommand& cmd) {
    // Predict ball position slightly ahead
    const float PREDICT_TIME = 0.1f;
    float predictX = ball.posX + ball.velX * PREDICT_TIME;
    float predictZ = ball.posZ + ball.velZ * PREDICT_TIME;

    // Clamp to valid field position
    clampToFieldBounds(predictX, predictZ);

    cmd.targetX = predictX;
    cmd.targetZ = predictZ;
    cmd.targetRotY = angleTo(robot.posX, robot.posZ, predictX, predictZ);
    cmd.dribbler = 0.8f;
    cmd.kick = 0.0f;
    cmd.chip = 0.0f;

    std::cerr << "  Action: Intercepting ball" << std::endl;
}

void BallControl::chaseBall(const RobotState& robot, const RobotState& ball,
    RobotCommand& cmd) {
    float targetX = ball.posX;
    float targetZ = ball.posZ;

    // Don't chase into goal areas
    clampToFieldBounds(targetX, targetZ);

    cmd.targetX = targetX;
    cmd.targetZ = targetZ;
    cmd.targetRotY = angleTo(robot.posX, robot.posZ, targetX, targetZ);
    cmd.dribbler = 0.9f;
    cmd.kick = 0.0f;
    cmd.chip = 0.0f;

    std::cerr << "  Action: Chasing ball" << std::endl;
}

void BallControl::dribbleTo(const RobotState& robot, const RobotState& ball,
    float targetX, float targetZ, RobotCommand& cmd) {
    // Move toward target while maintaining ball control
    float dirX = targetX - robot.posX;
    float dirZ = targetZ - robot.posZ;
    float len = std::sqrt(dirX * dirX + dirZ * dirZ);

    if (len > 0.01f) {
        dirX /= len;
        dirZ /= len;
    }

    float nextX = robot.posX + dirX * DRIBBLE_STEP;
    float nextZ = robot.posZ + dirZ * DRIBBLE_STEP;

    // Ensure we stay in bounds
    clampToFieldBounds(nextX, nextZ);

    cmd.targetX = nextX;
    cmd.targetZ = nextZ;

    // Rotate SLOWLY toward target direction while dribbling
    float targetAngle = angleTo(robot.posX, robot.posZ, targetX, targetZ);
    cmd.targetRotY = smoothRotation(robot.rotY, targetAngle, DRIBBLE_ROTATION_SPEED);

    cmd.dribbler = 1.0f;
    cmd.kick = 0.0f;
    cmd.chip = 0.0f;

    std::cerr << "  Action: Dribbling to (" << targetX << ", " << targetZ
        << ") - Rotating slowly to face goal" << std::endl;
}

/**
 * Dribble toward goal with controlled rotation (NEW VARIANT!)
 * Specifically for attacking - rotates slowly toward goal
 */
void BallControl::dribbleToGoal(const RobotState& robot, const RobotState& ball,
    RobotCommand& cmd) {
    // Calculate direction toward goal
    const float GOAL_APPROACH_X = RIGHT_GOAL_X - 0.5f;  // Stop before area
    float dirX = GOAL_APPROACH_X - robot.posX;
    float dirZ = -robot.posZ * 0.3f;  // Slight adjustment toward center
    float len = std::sqrt(dirX * dirX + dirZ * dirZ);

    if (len > 0.01f) {
        dirX /= len;
        dirZ /= len;
    }

    // Small steps for better control
    float nextX = robot.posX + dirX * DRIBBLE_STEP;
    float nextZ = robot.posZ + dirZ * DRIBBLE_STEP;

    // Ensure we stay in bounds and out of goal area
    clampToFieldBounds(nextX, nextZ);

    cmd.targetX = nextX;
    cmd.targetZ = nextZ;

    // Slowly rotate to face goal center while dribbling
    float angleToGoal = angleTo(robot.posX, robot.posZ, RIGHT_GOAL_X, 0.0f);
    cmd.targetRotY = smoothRotation(robot.rotY, angleToGoal, DRIBBLE_ROTATION_SPEED);

    cmd.dribbler = 1.0f;
    cmd.kick = 0.0f;
    cmd.chip = 0.0f;

    float currentAngleDeg = robot.rotY * 180.0f / M_PI;
    float targetAngleDeg = angleToGoal * 180.0f / M_PI;

    std::cerr << "  Action: Dribbling to goal - Current angle: " << currentAngleDeg
        << "°, Target: " << targetAngleDeg << "° (rotating slowly)" << std::endl;
}

// ============================================================================
// PASSING ACTIONS (REMOVED - Simple attacker/defender strategy)
// ============================================================================

// ============================================================================
// SHOOTING ACTIONS
// ============================================================================

ActionResult Shooting::shootAtGoal(const RobotState& robot, const RobotState& ball,
    RobotCommand& cmd) {
    ActionResult result;
    result.success = false;

    float targetX = RIGHT_GOAL_X;
    float targetZ = 0.0f;

    float angleToGoal = angleTo(ball.posX, ball.posZ, targetX, targetZ);

    if (!canKickBall(robot, ball)) {
        float offset = 0.15f;
        Vec2 goalDir = getFacingVector(angleToGoal);

        cmd.targetX = ball.posX - goalDir.x * offset;
        cmd.targetZ = ball.posZ - goalDir.z * offset;
        cmd.targetRotY = angleToGoal;
        cmd.dribbler = 0.8f;
        cmd.kick = 0.0f;
        cmd.chip = 0.0f;

        result.description = "Positioning for shot";
        std::cerr << "  Action: Positioning for shot" << std::endl;
        return result;
    }

    float angleDiff = std::abs(angleDifference(robot.rotY, angleToGoal));

    if (angleDiff > 15.0f * DEG_TO_RAD) {
        cmd.targetX = robot.posX;
        cmd.targetZ = robot.posZ;
        cmd.targetRotY = angleToGoal;
        cmd.dribbler = 0.8f;
        cmd.kick = 0.0f;
        cmd.chip = 0.0f;

        result.description = "Aligning for shot";
        std::cerr << "  Action: Aligning for shot" << std::endl;
        return result;
    }

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

    std::cerr << "  Action: ⚽ SHOOTING! Power: " << power << std::endl;

    return result;
}

void Shooting::clearBall(const RobotState& robot, const RobotState& ball,
    RobotCommand& cmd) {
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
        float offset = 0.15f;
        Vec2 clearDir = getFacingVector(angleToTarget);

        cmd.targetX = ball.posX - clearDir.x * offset;
        cmd.targetZ = ball.posZ - clearDir.z * offset;
        cmd.targetRotY = angleToTarget;
        cmd.dribbler = 0.5f;
        cmd.kick = 0.0f;
        cmd.chip = 0.0f;

        std::cerr << "  Action: Positioning to clear" << std::endl;
    }
}

// ============================================================================
// POSITIONING ACTIONS
// ============================================================================

void Positioning::holdPosition(const RobotState& robot, RobotCommand& cmd) {
    cmd.targetX = robot.posX;
    cmd.targetZ = robot.posZ;
    cmd.targetRotY = robot.rotY;
    cmd.dribbler = 0.0f;
    cmd.kick = 0.0f;
    cmd.chip = 0.0f;
}

void Positioning::moveTo(const RobotState& robot, float targetX, float targetZ,
    RobotCommand& cmd) {
    // Ensure target is within valid bounds
    clampToFieldBounds(targetX, targetZ);

    cmd.targetX = targetX;
    cmd.targetZ = targetZ;
    cmd.targetRotY = angleTo(robot.posX, robot.posZ, targetX, targetZ);
    cmd.dribbler = 0.0f;
    cmd.kick = 0.0f;
    cmd.chip = 0.0f;
}

void Positioning::faceTowards(const RobotState& robot, float pointX, float pointZ,
    RobotCommand& cmd) {
    cmd.targetX = robot.posX;
    cmd.targetZ = robot.posZ;
    cmd.targetRotY = angleTo(robot.posX, robot.posZ, pointX, pointZ);
    cmd.dribbler = 0.0f;
    cmd.kick = 0.0f;
    cmd.chip = 0.0f;
}

/**
 * Move to position while facing a specific point
 */
void Positioning::moveToWhileFacing(const RobotState& robot,
    float targetX, float targetZ,
    float faceX, float faceZ,
    RobotCommand& cmd) {
    // Ensure target is within valid bounds
    clampToFieldBounds(targetX, targetZ);

    cmd.targetX = targetX;
    cmd.targetZ = targetZ;
    // Face toward the point while moving
    cmd.targetRotY = angleTo(robot.posX, robot.posZ, faceX, faceZ);
    cmd.dribbler = 0.0f;
    cmd.kick = 0.0f;
    cmd.chip = 0.0f;
}