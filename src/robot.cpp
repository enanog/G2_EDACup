/**
 * @file Robot.cpp
 * @brief Implementation of Robot class
 * @author Agustin Valenzuela, Alex Petersen, Dylan Frigerio, Enzo Fernandez Rosas
 * @copyright Copyright (c) 2025
 */

#include "robot.h"

#include <iostream>

#include "ball.h"
#include "geometry.h"

// ========================================================================
// CONFIGURATION CONSTANTS
// ========================================================================

const float SMOOTH_ROTATION_SPEED = 0.1f;
const float DRIBBLE_ROTATION_SPEED = 0.1f;
const float MOVEMENT_STEP = 0.12f;
const float DRIBBLE_STEP = 0.12f;

// Field boundaries with safety margin
const float FIELD_X_MIN = -FIELD_HALF_LENGTH + 0.05f;
const float FIELD_X_MAX = FIELD_HALF_LENGTH - 0.05f;
const float FIELD_Z_MIN = -FIELD_HALF_WIDTH + 0.05f;
const float FIELD_Z_MAX = FIELD_HALF_WIDTH - 0.05f;

// ============================================================================
// CONSTRUCTORS
// ============================================================================

Robot::Robot()
    : posX_(0),
      posY_(0),
      posZ_(0),
      rotX_(0),
      rotY_(0),
      rotZ_(0),
      velX_(0),
      velY_(0),
      velZ_(0),
      angVelX_(0),
      angVelY_(0),
      angVelZ_(0),
      targetX_(0),
      targetZ_(0),
      targetRotY_(0),
      dribbler_(0),
      kick_(0),
      chip_(0) {
}

Robot::Robot(float x, float y, float z)
    : posX_(x),
      posY_(y),
      posZ_(z),
      rotX_(0),
      rotY_(0),
      rotZ_(0),
      velX_(0),
      velY_(0),
      velZ_(0),
      angVelX_(0),
      angVelY_(0),
      angVelZ_(0),
      targetX_(x),
      targetZ_(z),
      targetRotY_(0),
      dribbler_(0),
      kick_(0),
      chip_(0) {
}

// ============================================================================
// STATE QUERIES
// ============================================================================

float Robot::distanceTo(const Robot& other) const {
    float dx = other.posX_ - posX_;
    float dz = other.posZ_ - posZ_;
    return std::sqrt(dx * dx + dz * dz);
}

float Robot::distanceTo(float x, float z) const {
    float dx = x - posX_;
    float dz = z - posZ_;
    return std::sqrt(dx * dx + dz * dz);
}

bool Robot::hasBallControl(const Ball& ball) const {
    float dist = distanceTo(ball.getPosX(), ball.getPosZ());
    return dist < BALL_CONTROL_DISTANCE;
}

bool Robot::canKickBall(const Ball& ball) const {
    if (!hasBallControl(ball))
        return false;

    float angleToBall = angleTo(posX_, posZ_, ball.getPosX(), ball.getPosZ());
    float angleDiff = std::abs(angleDifference(rotY_, angleToBall));

    return (angleDiff < 30.0f * DEG_TO_RAD);
}

bool Robot::hasClearPath(float targetX, float targetZ, const Robot& opp1, const Robot& opp2) const {
    const float BLOCKING_DISTANCE = 0.2f;

    float lineLen = distanceTo(targetX, targetZ);
    if (lineLen < 0.01f)
        return true;

    // Direction vector
    float dirX = (targetX - posX_) / lineLen;
    float dirZ = (targetZ - posZ_) / lineLen;

    // Lambda: calculate distance from opponent to path line
    auto distToLine = [&](const Robot& opp) -> float {
        if (!opp.isOnField())
            return 999.0f;

        float dx = opp.posX_ - posX_;
        float dz = opp.posZ_ - posZ_;
        float proj = dx * dirX + dz * dirZ;
        proj = std::clamp(proj, 0.0f, lineLen);

        float nearX = posX_ + dirX * proj;
        float nearZ = posZ_ + dirZ * proj;

        float distX = nearX - opp.posX_;
        float distZ = nearZ - opp.posZ_;
        return std::sqrt(distX * distX + distZ * distZ);
    };

    float dist1 = distToLine(opp1);
    float dist2 = distToLine(opp2);

    return (dist1 > BLOCKING_DISTANCE && dist2 > BLOCKING_DISTANCE);
}

void Robot::clampToFieldBounds(float& x, float& z) const {
    x = std::clamp(x, FIELD_X_MIN, FIELD_X_MAX);
    z = std::clamp(z, FIELD_Z_MIN, FIELD_Z_MAX);

    if (x < -FIELD_HALF_LENGTH + PENALTY_AREA_DEPTH) {
        if (std::fabs(z) < PENALTY_AREA_HALF_WIDTH) {
            x = -FIELD_HALF_LENGTH + PENALTY_AREA_DEPTH + 0.05f;
            if (z > 0)
                z = PENALTY_AREA_HALF_WIDTH + 0.05f;
            else
                z = -PENALTY_AREA_HALF_WIDTH - 0.05f;
        }
    } else if (x > FIELD_HALF_LENGTH - PENALTY_AREA_DEPTH) {
        if (std::fabs(z) < PENALTY_AREA_HALF_WIDTH) {
            x = -FIELD_HALF_LENGTH + PENALTY_AREA_DEPTH + 0.05f;
            if (z > 0)
                z = PENALTY_AREA_HALF_WIDTH + 0.05f;
            else
                z = -PENALTY_AREA_HALF_WIDTH - 0.05f;
        }
    }
}

// ============================================================================
// BASIC CONTROL ACTIONS
// ============================================================================

void Robot::resetControls() {
    dribbler_ = 0.0f;
    kick_ = 0.0f;
    chip_ = 0.0f;
}

void Robot::holdPosition() {
    targetX_ = posX_;
    targetZ_ = posZ_;
    targetRotY_ = rotY_;
    resetControls();
}

void Robot::moveTo(float targetX, float targetZ) {
    clampToFieldBounds(targetX, targetZ);
    targetX_ = targetX;
    targetZ_ = targetZ;
    // targetRotY_ = angleTo(posX_, posZ_, targetX, targetZ);
    resetControls();
}

void Robot::faceTowards(float pointX, float pointZ) {
    // targetX_ = posX_;
    // targetZ_ = posZ_;
    targetRotY_ = angleTo(posX_, posZ_, pointX, pointZ);
    // resetControls();
}

void Robot::moveToWhileFacing(float targetX, float targetZ, float faceX, float faceZ) {
    clampToFieldBounds(targetX, targetZ);

    targetX_ = targetX;
    targetZ_ = targetZ;
    targetRotY_ = angleTo(posX_, posZ_, faceX, faceZ);
    resetControls();
}

// ============================================================================
// BALL INTERACTION ACTIONS
// ============================================================================

void Robot::interceptBall(const Ball& ball) {
    const float PREDICT_TIME = 0.1f;

    // Predict ball position
    float predictX, predictZ;
    ball.predictPosition(PREDICT_TIME, predictX, predictZ);

    targetX_ = predictX;
    targetZ_ = predictZ;
    targetRotY_ = angleTo(posX_, posZ_, predictX, predictZ);
    dribbler_ = 0.8f;
    kick_ = 0.0f;
    chip_ = 0.0f;

    std::cerr << "  Action: Intercepting ball at (" << predictX << ", " << predictZ << ")"
              << std::endl;
}

void Robot::chaseBall(const Ball& ball) {
    float targetX = ball.getPosX();
    float targetZ = ball.getPosZ();

    clampToFieldBounds(targetX, targetZ);

    targetX_ = targetX;
    targetZ_ = targetZ;
    targetRotY_ = angleTo(posX_, posZ_, targetX, targetZ);
    dribbler_ = 1.0f;
    kick_ = 0.0f;
    chip_ = 0.0f;

    std::cerr << "  Action: Chasing ball" << std::endl;
}

void Robot::dribbleTo(float targetX, float targetZ) {
    // Calculate direction to target
    float dirX = targetX - posX_;
    float dirZ = targetZ - posZ_;
    float len = std::sqrt(dirX * dirX + dirZ * dirZ);

    if (len > 0.01f) {
        dirX /= len;
        dirZ /= len;
    }

    // Take small step
    float nextX = posX_ + dirX * DRIBBLE_STEP;
    float nextZ = posZ_ + dirZ * DRIBBLE_STEP;

    clampToFieldBounds(nextX, nextZ);

    targetX_ = nextX;
    targetZ_ = nextZ;

    dribbler_ = 1.0f;
    kick_ = 0.0f;
    chip_ = 0.0f;

    std::cerr << "  Action: Dribbling to (" << targetX << ", " << targetZ << ")" << std::endl;
}

void Robot::dribbleToGoal() {
    // Direction toward opponent's goal
    const float GOAL_APPROACH_X = RIGHT_GOAL_X - 0.5f;
    float dirX = GOAL_APPROACH_X - posX_;
    float dirZ = -posZ_ * 0.3f;  // Slight center adjustment
    float len = std::sqrt(dirX * dirX + dirZ * dirZ);

    if (len > 0.01f) {
        dirX /= len;
        dirZ /= len;
    }

    // Small controlled steps
    float nextX = posX_ + dirX * DRIBBLE_STEP;
    float nextZ = posZ_ + dirZ * DRIBBLE_STEP;

    clampToFieldBounds(nextX, nextZ);

    targetX_ = nextX;
    targetZ_ = nextZ;

    // Slowly rotate to face goal center
    float angleToGoal = angleTo(posX_, posZ_, RIGHT_GOAL_X, 0.0f);
    targetRotY_ = smoothRotation(rotY_, angleToGoal, DRIBBLE_ROTATION_SPEED);

    dribbler_ = 1.0f;
    kick_ = 0.0f;
    chip_ = 0.0f;

    std::cerr << "  Action: Dribbling to goal - Angle: " << (rotY_ * 180.0f / M_PI)
              << (angleToGoal * 180.0f / M_PI) << std::endl;
}

bool Robot::shootAtGoal(const Ball& ball) {
    float angleToGoal = angleTo(posX_, posZ_, RIGHT_GOAL_X, 0.0f);

    // Step 1: Position behind ball if needed
    if (!canKickBall(ball)) {
        const float OFFSET = 0.15f;
        float goalDirX = std::cos(angleToGoal);
        float goalDirZ = std::sin(angleToGoal);

        targetX_ = ball.getPosX() - goalDirX * OFFSET;
        targetZ_ = ball.getPosZ() - goalDirZ * OFFSET;
        targetRotY_ = angleToGoal;
        dribbler_ = 0.8f;
        kick_ = 0.0f;
        chip_ = 0.0f;

        std::cerr << "  Action: Positioning for shot" << std::endl;
        return false;
    }

    // Step 2: Align if needed
    float angleDiff = std::abs(angleDifference(rotY_, angleToGoal));
    if (angleDiff > 15.0f * DEG_TO_RAD) {
        targetX_ = posX_;
        targetZ_ = posZ_;
        targetRotY_ = angleToGoal;
        dribbler_ = 0.8f;
        kick_ = 0.0f;
        chip_ = 0.0f;

        std::cerr << "  Action: Aligning for shot (diff: " << (angleDiff * 180.0f / M_PI) << ")"
                  << std::endl;
        return false;
    }

    // Step 3: Execute shot
    float distToGoal = ball.distanceTo(RIGHT_GOAL_X, 0.0f);
    float power = std::clamp(distToGoal / 2.0f, 0.6f, 1.0f);

    targetX_ = ball.getPosX();
    targetZ_ = ball.getPosZ();
    targetRotY_ = angleToGoal;
    dribbler_ = 0.0f;
    kick_ = power;
    chip_ = 0.0f;

    std::cerr << "  Action: SHOOTING! Power: " << power << std::endl;
    return true;
}

void Robot::clearBall(const Ball& ball) {
    // Clear toward opponent's side, away from center
    float targetX = RIGHT_GOAL_X;
    float targetZ = (ball.getPosZ() > 0) ? -FIELD_HALF_WIDTH : FIELD_HALF_WIDTH;

    float angleToTarget = angleTo(posX_, posZ_, targetX, targetZ);

    if (canKickBall(ball)) {
        // Execute clear
        targetX_ = ball.getPosX();
        targetZ_ = ball.getPosZ();
        targetRotY_ = angleToTarget;
        dribbler_ = 0.0f;
        kick_ = 0.0f;
        chip_ = 1.0f;

        std::cerr << "  Action: CLEARING ball!" << std::endl;
    } else {
        // Position for clear
        const float OFFSET = 0.15f;
        float clearDirX = std::cos(angleToTarget);
        float clearDirZ = std::sin(angleToTarget);

        targetX_ = ball.getPosX() - clearDirX * OFFSET;
        targetZ_ = ball.getPosZ() - clearDirZ * OFFSET;
        targetRotY_ = angleToTarget;
        dribbler_ = 0.5f;
        kick_ = 0.0f;
        chip_ = 0.0f;

        std::cerr << "  Action: Positioning to clear" << std::endl;
    }
}
