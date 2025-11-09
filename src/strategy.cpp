/**
 * @file strategy.cpp
 * @brief EDACup 2025 WINNING STRATEGY - Dynamic Roles, Field Mapping, Passing
 * @author Agustin Valenzuela, Alex Petersen, Dylan Frigerio, Enzo Fernandez Rosas
 * @copyright Copyright (c) 2025
 *
 * COMPETITION STRATEGY:
 * - Dynamic role assignment (STRIKER/DEFENDER)
 * - Field mapping for tactical awareness
 * - Intelligent passing with line-of-sight checks
 * - Latency compensation (0.15s prediction)
 * - Penalty area avoidance (ZERO penalties)
 * - Chip/Kick decision based on obstacles
 *
 * RESEARCH INTEGRATION:
 * - Luke (1997): Co-evolved passing behaviors
 * - Nakashima: Rule-based action selection
 * - Suriani: Role switching coordination
 * - Browning: Latency compensation
 * - Dragone: Dynamic environment mapping
 */

#include "strategy.h"
#include "actions.h"
#include "geometry.h"
#include "constants.h"
#include <iostream>
#include <algorithm>
#include <cmath>

 // ============================================================================
 // COMPETITION CONSTANTS
 // ============================================================================

 // Penalty area dimensions (CRITICAL - NO ENTRY!)
const float PENALTY_AREA_DEPTH = 0.30f;      // 30cm from goal line
const float PENALTY_AREA_HALF_WIDTH = 0.40f; // ±40cm from center

// Latency compensation
const float PREDICTION_TIME = 0.15f; // 150ms latency

// Role assignment thresholds
const float ROLE_HYSTERESIS = 0.15f; // Prevent role flickering

const float MIN_PASS_DISTANCE = 0.25f;   // Allow closer passes
const float MAX_PASS_DISTANCE = 1.2f;    // Allow slightly longer passes
const float PASS_SAFETY_MARGIN = 0.25f;  // Slightly less strict obstacle clearance

// Shooting parameters
const float OPTIMAL_SHOT_DISTANCE = 0.4f;
const float MAX_SHOT_DISTANCE = 0.6f;
const float CHIP_OBSTACLE_THRESHOLD = 0.40f; // Use chip if rival within this

// Threat detection
const float DANGER_ZONE_RADIUS = 0.7f; // Radius around own goal

// ============================================================================
// FIELD MAPPING SYSTEM
// ============================================================================

/**
 * @brief Comprehensive field mapping for tactical decision-making
 *
 * Tracks positions, threats, opportunities, and penalty areas.
 * Inspired by Dragone's persistent mapping in dynamic environments.
 */
struct FieldMap {
    // Penalty area boundaries
    float ownPenaltyMinX;
    float ownPenaltyMaxX;
    float oppPenaltyMinX;
    float oppPenaltyMaxX;
    float penaltyMinZ;
    float penaltyMaxZ;

    // Predicted positions (latency compensation - Browning)
    Vec2 predictedBallPos;
    Vec2 predictedBallVel;

    // Tactical information
    float ballDistToOwnGoal;
    float ballDistToOppGoal;
    int activeOpponents;

    // Threat assessment
    float closestRivalToOwnGoal;
    const RobotState* closestRivalPtr;

    // Passing opportunities
    bool passLineIsClear;
    float passQuality; // 0.0 to 1.0

    /**
     * Initialize field map with current game state
     */
    void update(const GameState& state) {
        // Define penalty areas
        ownPenaltyMinX = LEFT_GOAL_X;
        ownPenaltyMaxX = LEFT_GOAL_X + PENALTY_AREA_DEPTH + 0.1f;
        oppPenaltyMinX = RIGHT_GOAL_X - (PENALTY_AREA_DEPTH + 0.1f);
        oppPenaltyMaxX = RIGHT_GOAL_X;
        penaltyMinZ = -PENALTY_AREA_HALF_WIDTH;
        penaltyMaxZ = PENALTY_AREA_HALF_WIDTH;

        // Predict ball position (latency compensation)
        predictedBallPos.x = state.ball.posX + state.ball.velX * PREDICTION_TIME;
        predictedBallPos.z = state.ball.posZ + state.ball.velZ * PREDICTION_TIME;
        predictedBallVel.x = state.ball.velX;
        predictedBallVel.z = state.ball.velZ;

        // Clamp predicted position to field
        predictedBallPos.x = std::clamp(predictedBallPos.x, -FIELD_HALF_LENGTH + 0.12f, FIELD_HALF_LENGTH - 0.12f);
        predictedBallPos.z = std::clamp(predictedBallPos.z, -FIELD_HALF_WIDTH + 0.12f, FIELD_HALF_WIDTH - 0.12f);

        // Calculate distances
        ballDistToOwnGoal = distance(state.ball.posX, state.ball.posZ, LEFT_GOAL_X, 0.0f);
        ballDistToOppGoal = distance(state.ball.posX, state.ball.posZ, RIGHT_GOAL_X, 0.0f);

        // Count active opponents
        activeOpponents = 0;
        if (state.rivalBot1.isOnField()) activeOpponents++;
        if (state.rivalBot2.isOnField()) activeOpponents++;

        // Find closest rival to own goal (threat assessment)
        closestRivalToOwnGoal = 999.0f;
        closestRivalPtr = nullptr;

        if (state.rivalBot1.isOnField()) {
            float dist = distance(state.rivalBot1.posX, state.rivalBot1.posZ, LEFT_GOAL_X, 0.0f);
            if (dist < closestRivalToOwnGoal) {
                closestRivalToOwnGoal = dist;
                closestRivalPtr = &state.rivalBot1;
            }
        }
        if (state.rivalBot2.isOnField()) {
            float dist = distance(state.rivalBot2.posX, state.rivalBot2.posZ, LEFT_GOAL_X, 0.0f);
            if (dist < closestRivalToOwnGoal) {
                closestRivalToOwnGoal = dist;
                closestRivalPtr = &state.rivalBot2;
            }
        }
    }

    /**
     * Check if position is inside own penalty area
     */
    bool inOwnPenaltyArea(float x, float z) const {
        return (x >= ownPenaltyMinX && x <= ownPenaltyMaxX &&
            z >= penaltyMinZ && z <= penaltyMaxZ);
    }

    /**
     * Check if position is inside opponent penalty area
     */
    bool inOppPenaltyArea(float x, float z) const {
        return (x >= oppPenaltyMinX && x <= oppPenaltyMaxX &&
            z >= penaltyMinZ && z <= penaltyMaxZ);
    }

    /**
     * Escape from penalty area to nearest safe position
     */
    Vec2 escapeFromPenaltyArea(float x, float z) const {
        Vec2 safe(x, z);

        if (inOwnPenaltyArea(x, z)) {
            // Push forward
            safe.x = ownPenaltyMaxX + 0.10f;
        }
        else if (inOppPenaltyArea(x, z)) {
            // Push backward
            safe.x = oppPenaltyMinX - 0.10f;
        }

        // Also escape Z if needed
        if (z < penaltyMinZ) {
            safe.z = penaltyMinZ - 0.10f;
        }
        else if (z > penaltyMaxZ) {
            safe.z = penaltyMaxZ + 0.10f;
        }

        return safe;
    }

    /**
     * Check if pass line is clear from passer to receiver
     * Returns true if no rivals block the path
     */
    bool isPassLineClear(const RobotState& passer, const RobotState& receiver,
        const RobotState& rival1, const RobotState& rival2) const {
        float passLen = distance(passer.posX, passer.posZ, receiver.posX, receiver.posZ);
        if (passLen < 0.01f) return false;

        Vec2 passDir((receiver.posX - passer.posX) / passLen,
            (receiver.posZ - passer.posZ) / passLen);

        // Lambda: distance from point to line segment
        auto distToPassLine = [&](const RobotState& rival) -> float {
            if (!rival.isOnField()) return 999.0f;

            float dx = rival.posX - passer.posX;
            float dz = rival.posZ - passer.posZ;
            float projection = dx * passDir.x + dz * passDir.z;
            projection = std::clamp(projection, 0.0f, passLen);

            float nearX = passer.posX + passDir.x * projection;
            float nearZ = passer.posZ + passDir.z * projection;

            return distance(nearX, nearZ, rival.posX, rival.posZ);
            };

        float dist1 = distToPassLine(rival1);
        float dist2 = distToPassLine(rival2);

        return (dist1 > PASS_SAFETY_MARGIN && dist2 > PASS_SAFETY_MARGIN);
    }

    /**
     * Calculate pass quality score (0.0 = bad, 1.0 = excellent)
     * Based on: distance, clearance, receiver position advantage
     */
    float calculatePassQuality(const RobotState& passer, const RobotState& receiver,
        const RobotState& rival1, const RobotState& rival2) const {
        if (!receiver.isOnField()) return 0.0f;

        float dist = distance(passer.posX, passer.posZ, receiver.posX, receiver.posZ);

        // Bad if too close or too far
        if (dist < MIN_PASS_DISTANCE || dist > MAX_PASS_DISTANCE) return 0.0f;

        // Check clearance
        if (!isPassLineClear(passer, receiver, rival1, rival2)) return 0.0f;

        // Reward forward progress
        float forwardAdvantage = (receiver.posX - passer.posX) / FIELD_HALF_LENGTH;
        forwardAdvantage = std::clamp(forwardAdvantage, 0.0f, 1.0f);

        // Distance score (prefer medium distance)
        float distScore = 1.0f - std::abs(dist - 1.0f) / MAX_PASS_DISTANCE;
        distScore = std::clamp(distScore, 0.0f, 1.0f);

        // Combined quality
        return 0.4f * distScore + 0.6f * forwardAdvantage + 0.2f;
    }

    /**
     * Check if rival blocks direct shot to goal
     */
    bool isShootingPathBlocked(const RobotState& shooter,
        const RobotState& rival1,
        const RobotState& rival2) const {
        float shotLen = distance(shooter.posX, shooter.posZ, RIGHT_GOAL_X, 0.0f);
        if (shotLen < 0.01f) return false;

        Vec2 shotDir((RIGHT_GOAL_X - shooter.posX) / shotLen,
            (0.0f - shooter.posZ) / shotLen);

        auto distToShotLine = [&](const RobotState& rival) -> float {
            if (!rival.isOnField()) return 999.0f;

            float dx = rival.posX - shooter.posX;
            float dz = rival.posZ - shooter.posZ;
            float projection = dx * shotDir.x + dz * shotDir.z;

            // Only check rivals in front
            if (projection < 0.0f) return 999.0f;

            projection = std::clamp(projection, 0.0f, shotLen);

            float nearX = shooter.posX + shotDir.x * projection;
            float nearZ = shooter.posZ + shotDir.z * projection;

            return distance(nearX, nearZ, rival.posX, rival.posZ);
            };

        float dist1 = distToShotLine(rival1);
        float dist2 = distToShotLine(rival2);

        return (dist1 < CHIP_OBSTACLE_THRESHOLD || dist2 < CHIP_OBSTACLE_THRESHOLD);
    }
};

// ============================================================================
// ROLE ASSIGNMENT (Dynamic - Suriani's coordination)
// ============================================================================

/**
 * @brief Assign STRIKER/DEFENDER roles dynamically
 *
 * STRIKER: Closest to ball OR has ball control (with hysteresis)
 * DEFENDER: Other robot
 *
 * Inspired by Suriani's role switching with symbolic coordination.
 */
RoleAssignment assignRoles(const GameState& state, const RoleAssignment& prevRoles) {
    RoleAssignment roles;

    // Handle off-field robots
    bool bot1Active = state.homeBot1.isOnField();
    bool bot2Active = state.homeBot2.isOnField();

    if (!bot1Active && !bot2Active) {
        roles.bot1Role = Role::OFFFIELD;
        roles.bot2Role = Role::OFFFIELD;
        return roles;
    }

    if (!bot1Active) {
        roles.bot1Role = Role::OFFFIELD;
        roles.bot2Role = Role::DEFENDER;
        return roles;
    }
    if (!bot2Active) {
        roles.bot1Role = Role::DEFENDER;
        roles.bot2Role = Role::OFFFIELD;
        return roles;
    }

    // Calculate distances to predicted ball position
    FieldMap map;
    map.update(state);
  
    float dist1 = distance(state.homeBot1.posX, state.homeBot1.posZ,
        map.predictedBallPos.x, map.predictedBallPos.z);
    float dist2 = distance(state.homeBot2.posX, state.homeBot2.posZ,
        map.predictedBallPos.x, map.predictedBallPos.z);

    // Check ball control
    bool bot1HasBall = hasBallControl(state.homeBot1, state.ball);
    bool bot2HasBall = hasBallControl(state.homeBot2, state.ball);

    // Rule 1: If a robot has ball control, it's STRIKER
    if (bot1HasBall) {
        roles.bot1Role = Role::STRIKER;
        roles.bot2Role = Role::DEFENDER;
        return roles;
    }
    if (bot2HasBall) {
        roles.bot1Role = Role::DEFENDER;
        roles.bot2Role = Role::STRIKER;
        return roles;
    }

    // Rule 2: Closest to ball is STRIKER (with hysteresis)
    if (prevRoles.bot1Role == Role::STRIKER) {
        // Bot1 was striker, keep it unless Bot2 is significantly closer
        if (dist2 < dist1 - ROLE_HYSTERESIS) {
            roles.bot1Role = Role::DEFENDER;
            roles.bot2Role = Role::STRIKER;
        }
        else {
            roles.bot1Role = Role::STRIKER;
            roles.bot2Role = Role::DEFENDER;
        }
    }
    else {
        // Bot2 was striker, keep it unless Bot1 is significantly closer
        if (dist1 < dist2 - ROLE_HYSTERESIS) {
            roles.bot1Role = Role::STRIKER;
            roles.bot2Role = Role::DEFENDER;
        }
        else {
            roles.bot1Role = Role::DEFENDER;
            roles.bot2Role = Role::STRIKER;
        }
    }

    return roles;
}

// ============================================================================
// STRIKER LOGIC (Attack, Shoot, Pass)
// ============================================================================

/**
 * @brief STRIKER behavior: aggressive ball control, shooting, passing
 *
 * Decision tree:
 * 1. No ball → Intercept (with prediction)
 * 2. Has ball → Check shot/pass/dribble
 *    a. Clear shot & good distance → SHOOT (chip if blocked)
 *    b. Teammate better positioned → PASS
 *    c. Otherwise → DRIBBLE safely
 */
void strikerLogic(const RobotState& striker, const RobotState& defender,
    const GameState& state, const FieldMap& map, RobotCommand& cmd) {

    std::cerr << "\n>> STRIKER BEHAVIOR <<" << std::endl;

    // Check if striker is off-field
    if (!striker.isOnField()) {
        std::cerr << " STRIKER OFF-FIELD (Y=" << striker.posY << ") - HOLDING" << std::endl;
        cmd.targetX = striker.posX;
        cmd.targetZ = striker.posZ;
        cmd.targetRotY = striker.rotY;
        cmd.dribbler = cmd.kick = cmd.chip = 0.0f;
        return;
    }

    // STATE 1: Don't have ball control → INTERCEPT
    if (!hasBallControl(striker, state.ball)) {
        std::cerr << "  No ball control - INTERCEPTING" << std::endl;

        // Use predicted ball position (latency compensation)
        float targetX = map.predictedBallPos.x;
        float targetZ = map.predictedBallPos.z;

        // Avoid penalty areas
        if (map.inOwnPenaltyArea(targetX, targetZ) || map.inOppPenaltyArea(targetX, targetZ)) {
            Vec2 safe = map.escapeFromPenaltyArea(targetX, targetZ);
            targetX = safe.x;
            targetZ = safe.z;
        }

        cmd.targetX = targetX;
        cmd.targetZ = targetZ;
        cmd.targetRotY = angleTo(striker.posX, striker.posZ, targetX, targetZ);
        cmd.dribbler = 0.9f;
        cmd.kick = 0.0f;
        cmd.chip = 0.0f;

        std::cerr << " Moving to predicted ball: (" << targetX << ", " << targetZ << ")" << std::endl;
        return;
    }

    // STATE 2: Have ball control → DECIDE ACTION
    std::cerr << "  Ball control acquired" << std::endl;

    float distToGoal = distance(state.ball.posX, state.ball.posZ, RIGHT_GOAL_X, 0.0f);
    std::cerr << "  Distance to goal: " << distToGoal << "m" << std::endl;

    // OPTION A: SHOOT if in good position
    if (distToGoal < 0.5f && distToGoal > 0.25f) {
        // Check if aligned
        float angleToGoal = angleTo(state.ball.posX, state.ball.posZ, RIGHT_GOAL_X, 0.0f);
        float angleDiff = std::abs(angleDifference(striker.rotY, angleToGoal));

        if (angleDiff < 20.0f * DEG_TO_RAD) {
            // Decide: KICK or CHIP?
            bool blocked = map.isShootingPathBlocked(striker, state.rivalBot1, state.rivalBot2);

            if (blocked && distToGoal > 0.6f) {
                // Use CHIP to go over rival
                std::cerr << "  CHIP SHOT (rival blocking)" << std::endl;
                cmd.targetX = state.ball.posX;
                cmd.targetZ = state.ball.posZ;
                cmd.targetRotY = angleToGoal;
                cmd.dribbler = 0.0f;
                cmd.kick = 0.0f;
                cmd.chip = std::clamp(distToGoal / 3.0f, 0.5f, 0.9f);
                return;
            }
            else {
                // Use KICK
                std::cerr << "  KICK SHOT" << std::endl;
                cmd.targetX = state.ball.posX;
                cmd.targetZ = state.ball.posZ;
                cmd.targetRotY = angleToGoal;
                cmd.dribbler = 0.0f;
                cmd.kick = std::clamp(distToGoal / 2.0f, 0.6f, 1.0f);
                cmd.chip = 0.0f;
                return;
            }
        }
        else {
            // Align first
            std::cerr << "  Aligning for shot (angle diff: " << (angleDiff * 180.0f / M_PI) << "°)" << std::endl;
            cmd.targetX = striker.posX;
            cmd.targetZ = striker.posZ;
            cmd.targetRotY = angleToGoal;
            cmd.dribbler = 0.8f;
            cmd.kick = 0.0f;
            cmd.chip = 0.0f;
            return;
        }
    }

    // OPTION B: PASS if teammate better positioned
    if (defender.isOnField()) {
        float passQuality = map.calculatePassQuality(striker, defender,
            state.rivalBot1, state.rivalBot2);

        std::cerr << "  Pass quality to defender: " << passQuality << std::endl;

        if (passQuality > 0.35f) {
            // Execute pass (Luke's co-evolved passing)
            std::cerr << "  PASSING to defender" << std::endl;

            float angleToDefender = angleTo(state.ball.posX, state.ball.posZ,
                defender.posX, defender.posZ);
            float angleDiff = std::abs(angleDifference(striker.rotY, angleToDefender));

            if (angleDiff < 25.0f * DEG_TO_RAD) {
                // Execute pass
                float passDist = distance(striker.posX, striker.posZ, defender.posX, defender.posZ);
                float power = std::clamp(passDist / 3.0f, 0.4f, 0.8f);

                cmd.targetX = state.ball.posX;
                cmd.targetZ = state.ball.posZ;
                cmd.targetRotY = angleToDefender;
                cmd.dribbler = 0.0f;
                cmd.kick = power;
                cmd.chip = 0.0f;
                return;
            }
            else {
                // Align for pass
                std::cerr << "  Aligning for pass" << std::endl;
                cmd.targetX = striker.posX;
                cmd.targetZ = striker.posZ;
                cmd.targetRotY = angleToDefender;
                cmd.dribbler = 0.8f;
                cmd.kick = 0.0f;
                cmd.chip = 0.0f;
                return;
            }
        }
    }

    // OPTION C: DRIBBLE toward goal (safe movement)
    std::cerr << "  Dribbling toward goal" << std::endl;

    const float GOAL_APPROACH_X = RIGHT_GOAL_X - 0.5f;
    float dirX = GOAL_APPROACH_X - striker.posX;
    float dirZ = -striker.posZ * 0.3f; // Slight correction toward center
    float len = std::sqrt(dirX * dirX + dirZ * dirZ);

    if (len > 0.01f) {
        dirX /= len;
        dirZ /= len;
    }

    const float DRIBBLE_SPEED = 0.04f;
    float nextX = striker.posX + dirX * DRIBBLE_SPEED;
    float nextZ = striker.posZ + dirZ * DRIBBLE_SPEED;

    // Avoid penalty areas
    if (map.inOppPenaltyArea(nextX, nextZ)) {
        Vec2 safe = map.escapeFromPenaltyArea(nextX, nextZ);
        nextX = safe.x;
        nextZ = safe.z;
    }

    // Clamp to field
    nextX = std::clamp(nextX, -FIELD_HALF_LENGTH + 0.15f, FIELD_HALF_LENGTH - 0.15f);
    nextZ = std::clamp(nextZ, -FIELD_HALF_WIDTH + 0.15f, FIELD_HALF_WIDTH - 0.15f);

    cmd.targetX = nextX;
    cmd.targetZ = nextZ;

    float angleToGoal = angleTo(striker.posX, striker.posZ, RIGHT_GOAL_X, 0.0f);
    cmd.targetRotY = smoothRotation(striker.rotY, angleToGoal, 0.04f);

    cmd.dribbler = 1.0f;
    cmd.kick = 0.0f;
    cmd.chip = 0.0f;
}

// ============================================================================
// DEFENDER LOGIC (Defense, Mark, Support)
// ============================================================================

/**
 * @brief DEFENDER behavior: protect goal, mark threats, clear danger
 *
 * Decision tree:
 * 1. Ball in danger zone → CLEAR
 * 2. Has ball control → PASS or CLEAR
 * 3. Striker passing → RECEIVE
 * 4. Otherwise → MARK closest rival to own goal
 */
void defenderLogic(const RobotState& defender, const RobotState& striker,
    const GameState& state, const FieldMap& map, RobotCommand& cmd) {

    std::cerr << "\n>> DEFENDER BEHAVIOR <<" << std::endl;

    // Check if defender is off-field
    if (!defender.isOnField()) {
        std::cerr << "  DEFENDER OFF-FIELD (Y=" << defender.posY << ") - HOLDING" << std::endl;
        cmd.targetX = striker.posX;
        cmd.targetZ = striker.posZ;
        cmd.targetRotY = striker.rotY;
        cmd.dribbler = cmd.kick = cmd.chip = 0.0f;
        return;
    }

    // STATE 1: CRITICAL DANGER - Ball near own goal
    if (map.ballDistToOwnGoal < DANGER_ZONE_RADIUS) {
        std::cerr << "  DANGER! Ball near own goal (" << map.ballDistToOwnGoal << "m)" << std::endl;

        float distToBall = distance(defender.posX, defender.posZ, state.ball.posX, state.ball.posZ);

        if (distToBall < 0.5f) {
            // Close enough to clear
            std::cerr << "  CLEARING ball" << std::endl;

            float clearX = RIGHT_GOAL_X;
            float clearZ = (state.ball.posZ > 0) ? FIELD_HALF_WIDTH : -FIELD_HALF_WIDTH;
            float angleToClear = angleTo(state.ball.posX, state.ball.posZ, clearX, clearZ);

            if (canKickBall(defender, state.ball)) {
                cmd.targetX = state.ball.posX;
                cmd.targetZ = state.ball.posZ;
                cmd.targetRotY = angleToClear;
                cmd.dribbler = 0.0f;
                cmd.kick = 1.0f;
                cmd.chip = 0.0f;
            }
            else {
                // Position to kick
                float offset = 0.15f;
                Vec2 clearDir = getFacingVector(angleToClear);
                cmd.targetX = state.ball.posX - clearDir.x * offset;
                cmd.targetZ = state.ball.posZ - clearDir.z * offset;
                cmd.targetRotY = angleToClear;
                cmd.dribbler = 0.5f;
                cmd.kick = 0.0f;
                cmd.chip = 0.0f;
            }
            return;
        }
        else {
            // Chase ball
            std::cerr << "  Chasing ball to clear" << std::endl;
            cmd.targetX = state.ball.posX;
            cmd.targetZ = state.ball.posZ;
            cmd.targetRotY = angleTo(defender.posX, defender.posZ, state.ball.posX, state.ball.posZ);
            cmd.dribbler = 0.9f;
            cmd.kick = 0.0f;
            cmd.chip = 0.0f;
            return;
        }
    }

    // STATE 2: Has ball control → PASS or CLEAR
    if (hasBallControl(defender, state.ball)) {
        std::cerr << "  Defender has ball control" << std::endl;

        // Try to pass to striker if viable
        if (striker.isOnField()) {
            float passQuality = map.calculatePassQuality(defender, striker,
                state.rivalBot1, state.rivalBot2);

            std::cerr << "  Pass quality to striker: " << passQuality << std::endl;

            if (passQuality > 0.45f) {
                // Execute pass
                std::cerr << " PASSING to striker" << std::endl;

                float angleToStriker = angleTo(state.ball.posX, state.ball.posZ,
                    striker.posX, striker.posZ);
                float angleDiff = std::abs(angleDifference(defender.rotY, angleToStriker));

                if (angleDiff < 25.0f * DEG_TO_RAD) {
                    float passDist = distance(defender.posX, defender.posZ, striker.posX, striker.posZ);
                    float power = std::clamp(passDist / 3.0f, 0.4f, 0.8f);

                    cmd.targetX = state.ball.posX;
                    cmd.targetZ = state.ball.posZ;
                    cmd.targetRotY = angleToStriker;
                    cmd.dribbler = 0.0f;
                    cmd.kick = power;
                    cmd.chip = 0.0f;
                    return;
                }
                else {
                    // Align for pass
                    cmd.targetX = defender.posX;
                    cmd.targetZ = defender.posZ;
                    cmd.targetRotY = angleToStriker;
                    cmd.dribbler = 0.8f;
                    cmd.kick = 0.0f;
                    cmd.chip = 0.0f;
                    return;
                }
            }
        }

        // Otherwise, clear ball
        std::cerr << "  CLEARING ball forward" << std::endl;

        float clearX = RIGHT_GOAL_X;
        float clearZ = (state.ball.posZ > 0) ? -FIELD_HALF_WIDTH : FIELD_HALF_WIDTH;
        float angleToClear = angleTo(state.ball.posX, state.ball.posZ, clearX, clearZ);

        cmd.targetX = state.ball.posX;
        cmd.targetZ = state.ball.posZ;
        cmd.targetRotY = angleToClear;
        cmd.dribbler = 0.0f;
        cmd.kick = 0.9f;
        cmd.chip = 0.0f;
        return;
    }

    // STATE 3: Striker is passing → RECEIVE
    bool strikerHasBall = striker.isOnField() && hasBallControl(striker, state.ball);
    if (strikerHasBall) {
        float ballSpeed = std::sqrt(state.ball.velX * state.ball.velX +
            state.ball.velZ * state.ball.velZ);

        // If ball is moving toward us, prepare to receive
        if (ballSpeed > 0.2f) {
            float ballToDefender = angleTo(state.ball.posX, state.ball.posZ,
                defender.posX, defender.posZ);
            float ballVelAngle = std::atan2(state.ball.velZ, state.ball.velX);
            float velDiff = std::abs(angleDifference(ballVelAngle, ballToDefender));

            if (velDiff < 45.0f * DEG_TO_RAD) {
                std::cerr << "  Receiving pass from striker" << std::endl;

                // Intercept predicted position
                cmd.targetX = map.predictedBallPos.x;
                cmd.targetZ = map.predictedBallPos.z;
                cmd.targetRotY = angleTo(defender.posX, defender.posZ,
                    map.predictedBallPos.x, map.predictedBallPos.z);
                cmd.dribbler = 1.0f;
                cmd.kick = 0.0f;
                cmd.chip = 0.0f;
                return;
            }
        }
    }

    // STATE 4: MARK closest rival to own goal (Nakashima's rule-based marking)
    if (map.closestRivalPtr != nullptr) {
        const RobotState& threat = *map.closestRivalPtr;

        std::cerr << "  Marking rival threat (dist to goal: "
            << map.closestRivalToOwnGoal << "m)" << std::endl;

        // Position between threat and own goal
        float markX = (threat.posX + LEFT_GOAL_X) * 0.55f;
        float markZ = threat.posZ * 0.5f;

        // Keep defender in defensive zone
        markX = std::max(markX, LEFT_GOAL_X + 0.40f); // Stay out of penalty area
        markX = std::min(markX, -0.2f); // Don't go too far forward

        markZ = std::clamp(markZ, -FIELD_HALF_WIDTH + 0.2f, FIELD_HALF_WIDTH - 0.2f);

        // Avoid penalty areas
        if (map.inOwnPenaltyArea(markX, markZ)) {
            Vec2 safe = map.escapeFromPenaltyArea(markX, markZ);
            markX = safe.x;
            markZ = safe.z;
        }

        cmd.targetX = markX;
        cmd.targetZ = markZ;
        cmd.targetRotY = angleTo(defender.posX, defender.posZ, threat.posX, threat.posZ);
        cmd.dribbler = 0.0f;
        cmd.kick = 0.0f;
        cmd.chip = 0.0f;

        std::cerr << "  Marking position: (" << markX << ", " << markZ << ")" << std::endl;
        return;
    }

    // STATE 5: FALLBACK - Default defensive position
    std::cerr << "  Default defensive positioning" << std::endl;

    float defX = (state.ball.posX + LEFT_GOAL_X) * 0.5f;
    float defZ = state.ball.posZ * 0.4f;

    defX = std::max(defX, LEFT_GOAL_X + 0.40f);
    defX = std::clamp(defX, -FIELD_HALF_LENGTH + 0.3f, -0.3f);
    defZ = std::clamp(defZ, -FIELD_HALF_WIDTH + 0.3f, FIELD_HALF_WIDTH - 0.3f);

    // Avoid penalty areas
    if (map.inOwnPenaltyArea(defX, defZ)) {
        Vec2 safe = map.escapeFromPenaltyArea(defX, defZ);
        defX = safe.x;
        defZ = safe.z;
    }

    cmd.targetX = defX;
    cmd.targetZ = defZ;
    cmd.targetRotY = angleTo(defender.posX, defender.posZ, state.ball.posX, state.ball.posZ);
    cmd.dribbler = 0.0f;
    cmd.kick = 0.0f;
    cmd.chip = 0.0f;
}

// ============================================================================
// MAIN STRATEGY FUNCTION
// ============================================================================

/**
 * @brief Main decision-making function for EDACup 2025
 *
 * Integrates:
 * - Field mapping (Dragone)
 * - Dynamic role assignment (Suriani)
 * - Latency compensation (Browning)
 * - Rule-based actions (Nakashima)
 * - Co-evolved passing (Luke)
 *
 * Target: 70% possession, 6 goals, 0 penalties
 */
void decideStrategy(GameState& state) {
    state.resetCommands();

    // ========================================================================
    // FRAME HEADER
    // ========================================================================
    std::cerr << "\n" << std::string(70, '=') << std::endl;
    std::cerr << "FRAME " << state.frameCount << " - EDACup 2025 Strategy" << std::endl;
    std::cerr << std::string(70, '=') << std::endl;

    // ========================================================================
    // FIELD MAPPING UPDATE
    // ========================================================================
    FieldMap fieldMap;
    fieldMap.update(state);

    std::cerr << "\n[FIELD MAPPING]" << std::endl;
    std::cerr << "  Ball: (" << state.ball.posX << ", " << state.ball.posZ << ")" << std::endl;
    std::cerr << "  Ball predicted: (" << fieldMap.predictedBallPos.x << ", "
        << fieldMap.predictedBallPos.z << ")" << std::endl;
    std::cerr << "  Ball velocity: (" << state.ball.velX << ", " << state.ball.velZ << ")" << std::endl;
    std::cerr << "  Dist to own goal: " << fieldMap.ballDistToOwnGoal << "m" << std::endl;
    std::cerr << "  Dist to opp goal: " << fieldMap.ballDistToOppGoal << "m" << std::endl;
    std::cerr << "  Active opponents: " << fieldMap.activeOpponents << "/2" << std::endl;

    if (fieldMap.closestRivalPtr != nullptr) {
        std::cerr << "  Closest rival to goal: " << fieldMap.closestRivalToOwnGoal
            << "m (threat level: " << (fieldMap.closestRivalToOwnGoal < 1.0f ? "HIGH" : "MEDIUM")
            << ")" << std::endl;
    }

    // ========================================================================
    // ROBOT STATUS
    // ========================================================================
    std::cerr << "\n[ROBOT STATUS]" << std::endl;
    std::cerr << "  Bot1: (" << state.homeBot1.posX << ", " << state.homeBot1.posZ
        << ") Y=" << state.homeBot1.posY
        << (state.homeBot1.isOnField() ? " [ACTIVE]" : " [OFF-FIELD]") << std::endl;
    std::cerr << "  Bot2: (" << state.homeBot2.posX << ", " << state.homeBot2.posZ
        << ") Y=" << state.homeBot2.posY
        << (state.homeBot2.isOnField() ? " [ACTIVE]" : " [OFF-FIELD]") << std::endl;

    // Opponent status
    std::cerr << "  Rival1: Y=" << state.rivalBot1.posY
        << (state.rivalBot1.isOnField() ? " [ACTIVE]" : " [OFF-FIELD]") << std::endl;
    std::cerr << "  Rival2: Y=" << state.rivalBot2.posY
        << (state.rivalBot2.isOnField() ? " [ACTIVE]" : " [OFF-FIELD]") << std::endl;

    // ========================================================================
    // ROLE ASSIGNMENT
    // ========================================================================
    static RoleAssignment prevRoles = { Role::STRIKER, Role::DEFENDER };
    RoleAssignment currentRoles = assignRoles(state, prevRoles);

    std::cerr << "\n[ROLE ASSIGNMENT]" << std::endl;
    std::cerr << "  Bot1: " << (currentRoles.bot1Role == Role::STRIKER ? "STRIKER" : "DEFENDER") << std::endl;
    std::cerr << "  Bot2: " << (currentRoles.bot2Role == Role::STRIKER ? "DEFENDER" : "STRIKER") << std::endl;

    if (currentRoles.bot1Role != prevRoles.bot1Role || currentRoles.bot2Role != prevRoles.bot2Role) {
        std::cerr << "  >>> ROLE SWITCH DETECTED <<<" << std::endl;
    }

    prevRoles = currentRoles;

    // =======================================================
// EXECUTE BEHAVIORS BY ROLE
// =======================================================

// Bot 1
    switch (currentRoles.bot1Role) {
    case Role::STRIKER:
        strikerLogic(state.homeBot1, state.homeBot2, state, fieldMap, state.bot1Cmd);
        break;
    case Role::DEFENDER:
        defenderLogic(state.homeBot1, state.homeBot2, state, fieldMap, state.bot1Cmd);
        break;
    case Role::OFFFIELD:
        std::cerr << "  Bot1 OFF-FIELD - HOLDING POSITION" << std::endl;
        state.bot1Cmd.targetX = state.homeBot1.posX;
        state.bot1Cmd.targetZ = state.homeBot1.posZ;
        state.bot1Cmd.targetRotY = state.homeBot1.rotY;
        state.bot1Cmd.dribbler = state.bot1Cmd.kick = state.bot1Cmd.chip = 0.0f;
        break;
    }

    // Bot 2
    switch (currentRoles.bot2Role) {
    case Role::STRIKER:
        strikerLogic(state.homeBot2, state.homeBot1, state, fieldMap, state.bot2Cmd);
        break;
    case Role::DEFENDER:
        defenderLogic(state.homeBot2, state.homeBot1, state, fieldMap, state.bot2Cmd);
        break;
    case Role::OFFFIELD:
        std::cerr << "  Bot2 OFF-FIELD - HOLDING POSITION" << std::endl;
        state.bot2Cmd.targetX = state.homeBot2.posX;
        state.bot2Cmd.targetZ = state.homeBot2.posZ;
        state.bot2Cmd.targetRotY = state.homeBot2.rotY;
        state.bot2Cmd.dribbler = state.bot2Cmd.kick = state.bot2Cmd.chip = 0.0f;
        break;
    }

    // ========================================================================
    // PENALTY AREA SAFETY CHECK (CRITICAL!)
    // ========================================================================
    std::cerr << "\n[PENALTY AREA CHECK]" << std::endl;

    bool bot1InPenalty = fieldMap.inOwnPenaltyArea(state.bot1Cmd.targetX, state.bot1Cmd.targetZ) ||
        fieldMap.inOppPenaltyArea(state.bot1Cmd.targetX, state.bot1Cmd.targetZ);

    bool bot2InPenalty = fieldMap.inOwnPenaltyArea(state.bot2Cmd.targetX, state.bot2Cmd.targetZ) ||
        fieldMap.inOppPenaltyArea(state.bot2Cmd.targetX, state.bot2Cmd.targetZ);

    if (bot1InPenalty) {
        std::cerr << "  Bot1 target in PENALTY AREA - ESCAPING" << std::endl;
        Vec2 safe = fieldMap.escapeFromPenaltyArea(state.bot1Cmd.targetX, state.bot1Cmd.targetZ);
        state.bot1Cmd.targetX = safe.x;
        state.bot1Cmd.targetZ = safe.z;
    }

    if (bot2InPenalty) {
        std::cerr << "  Bot2 target in PENALTY AREA - ESCAPING" << std::endl;
        Vec2 safe = fieldMap.escapeFromPenaltyArea(state.bot2Cmd.targetX, state.bot2Cmd.targetZ);
        state.bot2Cmd.targetX = safe.x;
        state.bot2Cmd.targetZ = safe.z;
    }

    if (!bot1InPenalty && !bot2InPenalty) {
        std::cerr << "  All targets safe from penalty areas" << std::endl;
    }

    // ========================================================================
    // FINAL COMMANDS
    // ========================================================================
    std::cerr << "\n[FINAL COMMANDS]" << std::endl;
    std::cerr << "  Bot1 → (" << state.bot1Cmd.targetX << ", " << state.bot1Cmd.targetZ
        << ") rotY=" << (state.bot1Cmd.targetRotY * 180.0f / M_PI) << "°"
        << " dribbler=" << state.bot1Cmd.dribbler
        << " kick=" << state.bot1Cmd.kick
        << " chip=" << state.bot1Cmd.chip << std::endl;

    std::cerr << "  Bot2 → (" << state.bot2Cmd.targetX << ", " << state.bot2Cmd.targetZ
        << ") rotY=" << (state.bot2Cmd.targetRotY * 180.0f / M_PI) << "°"
        << " dribbler=" << state.bot2Cmd.dribbler
        << " kick=" << state.bot2Cmd.kick
        << " chip=" << state.bot2Cmd.chip << std::endl;

    std::cerr << std::string(70, '=') << std::endl << std::endl;
}