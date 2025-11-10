/**
 * @file field.cpp
 * @brief Field mapping and tactical awareness implementation
 * @author Agustin Valenzuela, Alex Petersen, Dylan Frigerio, Enzo Fernandez Rosas
 * @copyright Copyright (c) 2025
 */

#include "field.h"

#include <algorithm>
#include <iostream>
#include <cmath>

#include "ball.h"
#include "constants.h"
#include "robot.h"

// ============================================================================
// CONSTRUCTOR
// ============================================================================

FieldMap::FieldMap()
    : ownPenaltyMinX(0.0f),
      ownPenaltyMaxX(0.0f),
      oppPenaltyMinX(0.0f),
      oppPenaltyMaxX(0.0f),
      penaltyMinZ(0.0f),
      penaltyMaxZ(0.0f),
      predictedBallX(0.0f),
      predictedBallZ(0.0f),
      predictedBallVelX(0.0f),
      predictedBallVelZ(0.0f),
      ballDistToOwnGoal(0.0f),
      ballDistToOppGoal(0.0f),
      isBallStuck(false),
      activeOpponents(0),
      closestRivalToOwnGoal(999.0f),
      closestRivalPtr(nullptr) {
}

// ============================================================================
// FIELD MAP UPDATE
// ============================================================================

void FieldMap::update(const Ball& ball, const Robot& rival1, const Robot& rival2) {
    // Field dimensions constants
    const float PREDICTION_TIME = 0.15f;

    // ========================================================================
    // DEFINE PENALTY AREAS
    // ========================================================================

    // Own penalty area (left goal)
    ownPenaltyMinX = LEFT_GOAL_X;
    ownPenaltyMaxX = LEFT_GOAL_X + PENALTY_AREA_DEPTH + 0.005f;

    // Opponent penalty area (right goal)
    oppPenaltyMinX = RIGHT_GOAL_X - (PENALTY_AREA_DEPTH + 0.005f);
    oppPenaltyMaxX = RIGHT_GOAL_X;

    // Z boundaries (same for both penalty areas)
    penaltyMinZ = -PENALTY_AREA_HALF_WIDTH;
    penaltyMaxZ = PENALTY_AREA_HALF_WIDTH;

    // ========================================================================
    // PREDICT BALL POSITION (LATENCY COMPENSATION)
    // ========================================================================

    ball.predictPosition(PREDICTION_TIME, predictedBallX, predictedBallZ);
    predictedBallVelX = ball.getVelX();
    predictedBallVelZ = ball.getVelZ();

    // ========================================================================
    // CALCULATE DISTANCES TO GOALS
    // ========================================================================

    ballDistToOwnGoal = ball.distanceTo(LEFT_GOAL_X, 0.0f);
    ballDistToOppGoal = ball.distanceTo(RIGHT_GOAL_X, 0.0f);

    // ========================================================================
    // COUNT ACTIVE OPPONENTS
    // ========================================================================

    activeOpponents = 0;
    if (rival1.isOnField())
        activeOpponents++;
    if (rival2.isOnField())
        activeOpponents++;

    // ========================================================================
    // FIND CLOSEST RIVAL TO OWN GOAL (THREAT ASSESSMENT)
    // ========================================================================

    closestRivalToOwnGoal = 999.0f;
    closestRivalPtr = nullptr;

    if (rival1.isOnField()) {
        float dist = rival1.distanceTo(Robot(LEFT_GOAL_X, 0, 0));
        if (dist < closestRivalToOwnGoal) {
            closestRivalToOwnGoal = dist;
            closestRivalPtr = &rival1;
        }
    }

    if (rival2.isOnField()) {
        float dist = rival2.distanceTo(Robot(LEFT_GOAL_X, 0, 0));
        if (dist < closestRivalToOwnGoal) {
            closestRivalToOwnGoal = dist;
            closestRivalPtr = &rival2;
        }
    }

    // ========================================================================
    // CHECK IF BALL IS STUCK (>5 SECONDS STATIONARY)
    // ========================================================================

    // Static variables to track ball state across frames
    static int ballStuckFrames = 0;
    static float lastBallPosX = 0.0f;
    static float lastBallPosZ = 0.0f;

    // Check if ball speed is very low (< 0.15 m/s)
    float speedSquared = ball.getVelX() * ball.getVelX() +
                         ball.getVelY() * ball.getVelY() +
                         ball.getVelZ() * ball.getVelZ();
    bool isSpeedLow = speedSquared < 0.01f;  // 0.10^2

    // Check if ball position hasn't changed significantly
    bool isPositionConstant = (std::abs(ball.getPosX() - lastBallPosX) < 0.005f) &&
                              (std::abs(ball.getPosZ() - lastBallPosZ) < 0.005f);

    // If ball is stationary, increment counter
    if (isSpeedLow || isPositionConstant) {
        ballStuckFrames++;

        // Mark as stuck after 5 seconds (250 frames at 50Hz)
        if (ballStuckFrames >= 250) {
            isBallStuck = true;
        }
    } else {
        // Ball is moving, reset counter
        ballStuckFrames = 0;
        isBallStuck = false;
    }

    // Update last known position
    lastBallPosX = ball.getPosX();
    lastBallPosZ = ball.getPosZ();
}

// ============================================================================
// PENALTY AREA CHECKS
// ============================================================================

bool FieldMap::inOwnPenaltyArea(float x, float z) const {
    return (x >= ownPenaltyMinX && x <= ownPenaltyMaxX && z >= penaltyMinZ && z <= penaltyMaxZ);
}

bool FieldMap::inOppPenaltyArea(float x, float z) const {
    return (x >= oppPenaltyMinX && x <= oppPenaltyMaxX && z >= penaltyMinZ && z <= penaltyMaxZ);
}

// ============================================================================
// SAFE POSITION ADJUSTMENT
// ============================================================================

void FieldMap::getSafePosition(float& x, float& z) const {
    // Push out of own penalty area
    if (inOwnPenaltyArea(x, z)) {
        x = ownPenaltyMaxX + 0.125f;
    }
    // Push out of opponent penalty area
    else if (inOppPenaltyArea(x, z)) {
        x = oppPenaltyMinX - 0.125f;
    }

    // Adjust Z if outside lateral bounds
    if (z < penaltyMinZ) {
        z = penaltyMinZ - 0.125f;
    } else if (z > penaltyMaxZ) {
        z = penaltyMaxZ + 0.125f;
    }
}

// ============================================================================
// PASS QUALITY ANALYSIS
// ============================================================================

bool FieldMap::isPassLineClear(const Robot& passer,
                               const Robot& receiver,
                               const Robot& rival1,
                               const Robot& rival2) const {
    return passer.hasClearPath(receiver.getPosX(), receiver.getPosZ(), rival1, rival2);
}

float FieldMap::calculatePassQuality(const Robot& passer,
                                     const Robot& receiver,
                                     const Robot& rival1,
                                     const Robot& rival2) const {
    // Cannot pass to off-field robot
    if (!receiver.isOnField())
        return 0.0f;

    float dist = passer.distanceTo(receiver);

    // Too close or too far - bad pass
    if (dist < 0.25f || dist > 1.2f)
        return 0.0f;

    // Check if path is clear
    if (!isPassLineClear(passer, receiver, rival1, rival2))
        return 0.0f;

    // ========================================================================
    // CALCULATE QUALITY COMPONENTS
    // ========================================================================

    // Reward forward progress (towards opponent goal)
    float forwardAdvantage = (receiver.getPosX() - passer.getPosX()) / FIELD_HALF_LENGTH;
    forwardAdvantage = std::clamp(forwardAdvantage, 0.0f, 1.0f);

    // Optimal distance is around 1.0m
    float distScore = 1.0f - std::abs(dist - 1.0f) / 1.2f;
    distScore = std::clamp(distScore, 0.0f, 1.0f);

    // Combined score with weights:
    // - 40% distance optimization
    // - 60% forward progress
    // - 20% baseline bonus for any valid pass
    std::cerr << "    Pass quality calculation: dist=" << dist << ", distScore=" << distScore
              << ", forwardAdvantage=" << forwardAdvantage << std::endl;
    return 0.5f * distScore + 0.5f * forwardAdvantage + 0.2f;
}

// ============================================================================
// SHOOTING PATH ANALYSIS
// ============================================================================

bool FieldMap::isShootingPathBlocked(const Robot& shooter,
                                     const Robot& rival1,
                                     const Robot& rival2) const {
    // Check if there's a clear path to the center of the opponent goal
    return !shooter.hasClearPath(RIGHT_GOAL_X, 0.0f, rival1, rival2);
}
