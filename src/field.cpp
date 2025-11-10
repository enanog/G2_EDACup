#include "field.h"

#include "ball.h"
#include "robot.h"

// ============================================================================
// FIELD MAP IMPLEMENTATION
// ============================================================================

FieldMap::FieldMap()
    : ownPenaltyMinX(0),
      ownPenaltyMaxX(0),
      oppPenaltyMinX(0),
      oppPenaltyMaxX(0),
      penaltyMinZ(0),
      penaltyMaxZ(0),
      predictedBallX(0),
      predictedBallZ(0),
      predictedBallVelX(0),
      predictedBallVelZ(0),
      ballDistToOwnGoal(0),
      ballDistToOppGoal(0),
      isballStuck(0),
      activeOpponents(0),
      closestRivalToOwnGoal(999.0f),
      closestRivalPtr(nullptr) {
}

void FieldMap::update(const Ball& ball, const Robot& rival1, const Robot& rival2) {
    const float PENALTY_AREA_DEPTH = 0.30f;
    const float PENALTY_AREA_HALF_WIDTH = 0.40f;
    const float PREDICTION_TIME = 0.15f;

    // Define penalty areas
    ownPenaltyMinX = LEFT_GOAL_X;
    ownPenaltyMaxX = LEFT_GOAL_X + PENALTY_AREA_DEPTH + 0.005f;
    oppPenaltyMinX = RIGHT_GOAL_X - (PENALTY_AREA_DEPTH + 0.005f);
    oppPenaltyMaxX = RIGHT_GOAL_X;
    penaltyMinZ = -PENALTY_AREA_HALF_WIDTH;
    penaltyMaxZ = PENALTY_AREA_HALF_WIDTH;

    // Predict ball position
    ball.predictPosition(PREDICTION_TIME, predictedBallX, predictedBallZ);
    predictedBallVelX = ball.getVelX();
    predictedBallVelZ = ball.getVelZ();

    // Calculate distances
    ballDistToOwnGoal = ball.distanceTo(LEFT_GOAL_X, 0.0f);
    ballDistToOppGoal = ball.distanceTo(RIGHT_GOAL_X, 0.0f);

    // Count active opponents
    activeOpponents = 0;
    if (rival1.isOnField())
        activeOpponents++;
    if (rival2.isOnField())
        activeOpponents++;

    // Find closest rival to own goal
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

    // Check if ball is stuck for more than 5 seconds
    static int ballStuckFrames = 0;
    static float lastBallPosX = 0.0f;
    static float lastBallPosZ = 0.0f;

    float ballSpeedSquare = ball.getVelX() * ball.getVelX() + ball.getVelZ() * ball.getVelZ();


    if (ballSpeedSquare < 0.0225f) {  // 0.15^2
        ballStuckFrames++;
        if (ballStuckFrames >= 50) {  // 5 seconds at 10 FPS
            isballStuck = true;
        }
    } else {
        ballStuckFrames = 0;
        isballStuck = false;
    }


    lastBallPosX = ball.getPosX();
    lastBallPosZ = ball.getPosZ();
}

bool FieldMap::inOwnPenaltyArea(float x, float z) const {
    return (x >= ownPenaltyMinX && x <= ownPenaltyMaxX && z >= penaltyMinZ && z <= penaltyMaxZ);
}

bool FieldMap::inOppPenaltyArea(float x, float z) const {
    return (x >= oppPenaltyMinX && x <= oppPenaltyMaxX && z >= penaltyMinZ && z <= penaltyMaxZ);
}

void FieldMap::getSafePosition(float& x, float& z) const {
    if (inOwnPenaltyArea(x, z)) {
        x = ownPenaltyMaxX + 0.10f;
    } else if (inOppPenaltyArea(x, z)) {
        x = oppPenaltyMinX - 0.10f;
    }

    if (z < penaltyMinZ) {
        z = penaltyMinZ - 0.10f;
    } else if (z > penaltyMaxZ) {
        z = penaltyMaxZ + 0.10f;
    }
}

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
    if (!receiver.isOnField())
        return 0.0f;

    float dist = passer.distanceTo(receiver);

    // Bad if too close or too far
    if (dist < 0.25f || dist > 1.2f)
        return 0.0f;

    // Check clearance
    if (!isPassLineClear(passer, receiver, rival1, rival2))
        return 0.0f;

    // Reward forward progress
    float forwardAdvantage = (receiver.getPosX() - passer.getPosX()) / FIELD_HALF_LENGTH;
    forwardAdvantage = std::clamp(forwardAdvantage, 0.0f, 1.0f);

    // Distance score
    float distScore = 1.0f - std::abs(dist - 1.0f) / 1.2f;
    distScore = std::clamp(distScore, 0.0f, 1.0f);

    return 0.4f * distScore + 0.6f * forwardAdvantage + 0.2f;
}

bool FieldMap::isShootingPathBlocked(const Robot& shooter,
                                     const Robot& rival1,
                                     const Robot& rival2) const {
    return !shooter.hasClearPath(RIGHT_GOAL_X, 0.0f, rival1, rival2);
}
