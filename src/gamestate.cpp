/**
 * @file GameState.cpp
 * @brief Implementation of game state manager and strategy
 * @author Agustin Valenzuela, Alex Petersen, Dylan Frigerio, Enzo Fernandez Rosas
 * @copyright Copyright (c) 2025
 */

#include "GameState.h"
#include <iostream>
#include <algorithm>
#include <cmath>

// ============================================================================
// FIELD MAP IMPLEMENTATION
// ============================================================================

FieldMap::FieldMap()
    : ownPenaltyMinX(0), ownPenaltyMaxX(0),
      oppPenaltyMinX(0), oppPenaltyMaxX(0),
      penaltyMinZ(0), penaltyMaxZ(0),
      predictedBallX(0), predictedBallZ(0),
      predictedBallVelX(0), predictedBallVelZ(0),
      ballDistToOwnGoal(0), ballDistToOppGoal(0),
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
    if (rival1.isOnField()) activeOpponents++;
    if (rival2.isOnField()) activeOpponents++;

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
}

bool FieldMap::inOwnPenaltyArea(float x, float z) const {
    return (x >= ownPenaltyMinX && x <= ownPenaltyMaxX &&
            z >= penaltyMinZ && z <= penaltyMaxZ);
}

bool FieldMap::inOppPenaltyArea(float x, float z) const {
    return (x >= oppPenaltyMinX && x <= oppPenaltyMaxX &&
            z >= penaltyMinZ && z <= penaltyMaxZ);
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

bool FieldMap::isPassLineClear(const Robot& passer, const Robot& receiver,
                                const Robot& rival1, const Robot& rival2) const {
    const float PASS_SAFETY_MARGIN = 0.25f;
    return passer.hasClearPath(receiver.getPosX(), receiver.getPosZ(), rival1, rival2);
}

float FieldMap::calculatePassQuality(const Robot& passer, const Robot& receiver,
                                     const Robot& rival1, const Robot& rival2) const {
    if (!receiver.isOnField()) return 0.0f;

    float dist = passer.distanceTo(receiver);

    // Bad if too close or too far
    if (dist < 0.25f || dist > 1.2f) return 0.0f;

    // Check clearance
    if (!isPassLineClear(passer, receiver, rival1, rival2)) return 0.0f;

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

// ============================================================================
// GAME STATE IMPLEMENTATION
// ============================================================================

GameState::GameState()
    : frameCount_(0) {
}

void GameState::reset() {
    frameCount_ = 0;
    homeBot1_.resetControls();
    homeBot2_.resetControls();
    currentRoles_ = RoleAssignment();
}

void GameState::parseFromJson(const json& message) {
    if (!message.contains("data")) {
        std::cerr << "Warning: Missing data field" << std::endl;
        return;
    }

    const json& data = message["data"];

    // Helper lambda to read robot/ball state
    auto readEntity = [&](const json& obj, float& posX, float& posY, float& posZ,
                          float& rotX, float& rotY, float& rotZ,
                          float& velX, float& velY, float& velZ) {
        try {
            if (obj.contains("position") && obj["position"].is_array() &&
                obj["position"].size() >= 3) {
                posX = obj["position"][0];
                posY = obj["position"][1];
                posZ = obj["position"][2];
            }

            if (obj.contains("rotation") && obj["rotation"].is_array() &&
                obj["rotation"].size() >= 3) {
                rotX = obj["rotation"][0];
                rotY = obj["rotation"][1];
                rotZ = obj["rotation"][2];
            }

            if (obj.contains("velocity") && obj["velocity"].is_array() &&
                obj["velocity"].size() >= 3) {
                velX = obj["velocity"][0];
                velY = obj["velocity"][1];
                velZ = obj["velocity"][2];
            }
        } catch (const std::exception& e) {
            std::cerr << "Error reading entity: " << e.what() << std::endl;
        }
    };

    // Read robots
    if (data.contains("homeBot1")) {
        float px, py, pz, rx, ry, rz, vx, vy, vz;
        readEntity(data["homeBot1"], px, py, pz, rx, ry, rz, vx, vy, vz);
        homeBot1_.setPosition(px, py, pz);
        homeBot1_.setRotation(rx, ry, rz);
        homeBot1_.setVelocity(vx, vy, vz);
    }

    if (data.contains("homeBot2")) {
        float px, py, pz, rx, ry, rz, vx, vy, vz;
        readEntity(data["homeBot2"], px, py, pz, rx, ry, rz, vx, vy, vz);
        homeBot2_.setPosition(px, py, pz);
        homeBot2_.setRotation(rx, ry, rz);
        homeBot2_.setVelocity(vx, vy, vz);
    }

    if (data.contains("rivalBot1")) {
        float px, py, pz, rx, ry, rz, vx, vy, vz;
        readEntity(data["rivalBot1"], px, py, pz, rx, ry, rz, vx, vy, vz);
        rivalBot1_.setPosition(px, py, pz);
        rivalBot1_.setRotation(rx, ry, rz);
        rivalBot1_.setVelocity(vx, vy, vz);
    }

    if (data.contains("rivalBot2")) {
        float px, py, pz, rx, ry, rz, vx, vy, vz;
        readEntity(data["rivalBot2"], px, py, pz, rx, ry, rz, vx, vy, vz);
        rivalBot2_.setPosition(px, py, pz);
        rivalBot2_.setRotation(rx, ry, rz);
        rivalBot2_.setVelocity(vx, vy, vz);
    }

    if (data.contains("ball")) {
        float px, py, pz, rx, ry, rz, vx, vy, vz;
        readEntity(data["ball"], px, py, pz, rx, ry, rz, vx, vy, vz);
        ball_.setPosition(px, py, pz);
        ball_.setRotation(rx, ry, rz);
        ball_.setVelocity(vx, vy, vz);
    }
}

json GameState::createCommandJson() const {
    json message = {
        {"type", "set"},
        {"data", {
            {"homeBot1", {
                {"positionXZ", {homeBot1_.getTargetX(), homeBot1_.getTargetZ()}},
                {"rotationY", homeBot1_.getTargetRotY()},
                {"dribbler", homeBot1_.getDribbler()},
                {"kick", homeBot1_.getKick()},
                {"chip", homeBot1_.getChip()}
            }},
            {"homeBot2", {
                {"positionXZ", {homeBot2_.getTargetX(), homeBot2_.getTargetZ()}},
                {"rotationY", homeBot2_.getTargetRotY()},
                {"dribbler", homeBot2_.getDribbler()},
                {"kick", homeBot2_.getKick()},
                {"chip", homeBot2_.getChip()}
            }}
        }}
    };
    return message;
}

// ============================================================================
// ROLE ASSIGNMENT
// ============================================================================

void GameState::assignRoles() {
    // Handle off-field robots
    bool bot1Active = homeBot1_.isOnField();
    bool bot2Active = homeBot2_.isOnField();

    if (!bot1Active && !bot2Active) {
        currentRoles_.bot1Role = Role::OFFFIELD;
        currentRoles_.bot2Role = Role::OFFFIELD;
        return;
    }

    if (!bot1Active) {
        currentRoles_.bot1Role = Role::OFFFIELD;
        currentRoles_.bot2Role = Role::DEFENDER;
        return;
    }
    if (!bot2Active) {
        currentRoles_.bot1Role = Role::DEFENDER;
        currentRoles_.bot2Role = Role::OFFFIELD;
        return;
    }

    // Calculate distances to predicted ball
    float dist1 = homeBot1_.distanceTo(Robot(fieldMap_.predictedBallX, 0, fieldMap_.predictedBallZ));
    float dist2 = homeBot2_.distanceTo(Robot(fieldMap_.predictedBallX, 0, fieldMap_.predictedBallZ));

    // Check ball control
    bool bot1HasBall = homeBot1_.hasBallControl(ball_);
    bool bot2HasBall = homeBot2_.hasBallControl(ball_);

    // Rule 1: Robot with ball is STRIKER
    if (bot1HasBall) {
        currentRoles_.bot1Role = Role::STRIKER;
        currentRoles_.bot2Role = Role::DEFENDER;
        return;
    }
    if (bot2HasBall) {
        currentRoles_.bot1Role = Role::DEFENDER;
        currentRoles_.bot2Role = Role::STRIKER;
        return;
    }

    // Rule 2: Closest to ball is STRIKER (with hysteresis)
    if (currentRoles_.bot1Role == Role::STRIKER) {
        if (dist2 < dist1 - ROLE_HYSTERESIS) {
            currentRoles_.bot1Role = Role::DEFENDER;
            currentRoles_.bot2Role = Role::STRIKER;
        }
    } else {
        if (dist1 < dist2 - ROLE_HYSTERESIS) {
            currentRoles_.bot1Role = Role::STRIKER;
            currentRoles_.bot2Role = Role::DEFENDER;
        }
    }
}

// ============================================================================
// STRIKER LOGIC
// ============================================================================

void GameState::executeStrikerLogic(Robot& striker, const Robot& defender) {
    std::cerr << "\n>> STRIKER BEHAVIOR <<" << std::endl;

    if (!striker.isOnField()) {
        std::cerr << "  STRIKER OFF-FIELD - HOLDING" << std::endl;
        striker.holdPosition();
        return;
    }

    // STATE 1: No ball control → INTERCEPT
    if (!striker.hasBallControl(ball_)) {
        std::cerr << "  No ball control - INTERCEPTING" << std::endl;
        
        float targetX = fieldMap_.predictedBallX;
        float targetZ = fieldMap_.predictedBallZ;

        // Avoid penalty areas
        if (fieldMap_.inOwnPenaltyArea(targetX, targetZ) || 
            fieldMap_.inOppPenaltyArea(targetX, targetZ)) {
            fieldMap_.getSafePosition(targetX, targetZ);
        }

        striker.moveTo(targetX, targetZ);
        // Manual dribbler control for interception
        const_cast<Robot&>(striker).resetControls();
        return;
    }

    // STATE 2: Has ball control
    std::cerr << "  Ball control acquired" << std::endl;

    float distToGoal = ball_.distanceTo(RIGHT_GOAL_X, 0.0f);
    std::cerr << "  Distance to goal: " << distToGoal << "m" << std::endl;

    // OPTION A: SHOOT if in good position
    if (distToGoal < 0.5f && distToGoal > 0.25f) {
        bool shotExecuted = striker.shootAtGoal(ball_);
        
        if (shotExecuted) {
            // Check if blocked and use chip if needed
            bool blocked = fieldMap_.isShootingPathBlocked(striker, rivalBot1_, rivalBot2_);
            if (blocked && distToGoal > 0.6f) {
                std::cerr << "  Using CHIP over obstacle" << std::endl;
                // Modify to chip shot (manual override)
            }
        }
        return;
    }

    // OPTION B: PASS if teammate better positioned
    if (defender.isOnField()) {
        float passQuality = fieldMap_.calculatePassQuality(striker, defender, 
                                                          rivalBot1_, rivalBot2_);
        
        std::cerr << "  Pass quality: " << passQuality << std::endl;

        if (passQuality > 0.35f) {
            std::cerr << "  PASSING to defender" << std::endl;
            // Execute pass logic (simplified - would need custom implementation)
            float dist = striker.distanceTo(defender);
            // Would need to align and kick
            return;
        }
    }

    // OPTION C: DRIBBLE toward goal
    std::cerr << "  Dribbling toward goal" << std::endl;
    striker.dribbleToGoal(ball_);
}

// ============================================================================
// DEFENDER LOGIC
// ============================================================================

void GameState::executeDefenderLogic(Robot& defender, const Robot& striker) {
    std::cerr << "\n>> DEFENDER BEHAVIOR <<" << std::endl;

    if (!defender.isOnField()) {
        std::cerr << "  DEFENDER OFF-FIELD - HOLDING" << std::endl;
        defender.holdPosition();
        return;
    }

    // STATE 1: DANGER - Ball near own goal
    if (fieldMap_.ballDistToOwnGoal < DANGER_ZONE_RADIUS) {
        std::cerr << "  DANGER! Ball near goal" << std::endl;

        float distToBall = ball_.distanceTo(defender);
        
        if (distToBall < 0.5f) {
            std::cerr << "  CLEARING ball" << std::endl;
            defender.clearBall(ball_);
            return;
        } else {
            std::cerr << "  Chasing ball to clear" << std::endl;
            defender.chaseBall(ball_);
            return;
        }
    }

    // STATE 2: Has ball control → PASS or CLEAR
    if (defender.hasBallControl(ball_)) {
        std::cerr << "  Defender has ball" << std::endl;

        if (striker.isOnField()) {
            float passQuality = fieldMap_.calculatePassQuality(defender, striker,
                                                              rivalBot1_, rivalBot2_);
            
            if (passQuality > 0.45f) {
                std::cerr << "  PASSING to striker" << std::endl;
                // Execute pass
                return;
            }
        }

        std::cerr << "  CLEARING forward" << std::endl;
        defender.clearBall(ball_);
        return;
    }

    // STATE 3: MARK closest rival
    if (fieldMap_.closestRivalPtr != nullptr) {
        const Robot& threat = *fieldMap_.closestRivalPtr;
        
        std::cerr << "  Marking rival threat" << std::endl;

        // Position between threat and own goal
        float markX = (threat.getPosX() + LEFT_GOAL_X) * 0.55f;
        float markZ = threat.getPosZ() * 0.5f;

        markX = std::max(markX, LEFT_GOAL_X + 0.40f);
        markX = std::min(markX, -0.2f);
        markZ = std::clamp(markZ, -FIELD_HALF_WIDTH + 0.2f, FIELD_HALF_WIDTH - 0.2f);

        if (fieldMap_.inOwnPenaltyArea(markX, markZ)) {
            fieldMap_.getSafePosition(markX, markZ);
        }

        defender.moveToWhileFacing(markX, markZ, threat.getPosX(), threat.getPosZ());
        return;
    }

    // STATE 4: DEFAULT defensive position
    std::cerr << "  Default defensive positioning" << std::endl;

    float defX = (ball_.getPosX() + LEFT_GOAL_X) * 0.5f;
    float defZ = ball_.getPosZ() * 0.4f;

    defX = std::max(defX, LEFT_GOAL_X + 0.40f);
    defZ = std::clamp(defZ, -FIELD_HALF_WIDTH + 0.3f, FIELD_HALF_WIDTH - 0.3f);

    if (fieldMap_.inOwnPenaltyArea(defX, defZ)) {
        fieldMap_.getSafePosition(defX, defZ);
    }

    defender.moveToWhileFacing(defX, defZ, ball_.getPosX(), ball_.getPosZ());
}

// ============================================================================
// MAIN UPDATE
// ============================================================================

void GameState::update() {
    // Reset controls
    homeBot1_.resetControls();
    homeBot2_.resetControls();

    std::cerr << "\n" << std::string(70, '=') << std::endl;
    std::cerr << "FRAME " << frameCount_ << " - Strategy Update" << std::endl;
    std::cerr << std::string(70, '=') << std::endl;

    // Update field map
    fieldMap_.update(ball_, rivalBot1_, rivalBot2_);

    std::cerr << "\n[FIELD STATE]" << std::endl;
    std::cerr << "  Ball: (" << ball_.getPosX() << ", " << ball_.getPosZ() << ")" << std::endl;
    std::cerr << "  Ball predicted: (" << fieldMap_.predictedBallX 
              << ", " << fieldMap_.predictedBallZ << ")" << std::endl;
    std::cerr << "  Dist to own goal: " << fieldMap_.ballDistToOwnGoal << "m" << std::endl;
    std::cerr << "  Active opponents: " << fieldMap_.activeOpponents << "/2" << std::endl;

    // Assign roles
    assignRoles();

    std::cerr << "\n[ROLES]" << std::endl;
    std::cerr << "  Bot1: " << (currentRoles_.bot1Role == Role::STRIKER ? "STRIKER" : "DEFENDER") << std::endl;
    std::cerr << "  Bot2: " << (currentRoles_.bot2Role == Role::STRIKER ? "STRIKER" : "DEFENDER") << std::endl;

    // Execute behaviors
    if (currentRoles_.bot1Role == Role::STRIKER) {
        executeStrikerLogic(homeBot1_, homeBot2_);
    } else if (currentRoles_.bot1Role == Role::DEFENDER) {
        executeDefenderLogic(homeBot1_, homeBot2_);
    }

    if (currentRoles_.bot2Role == Role::STRIKER) {
        executeStrikerLogic(homeBot2_, homeBot1_);
    } else if (currentRoles_.bot2Role == Role::DEFENDER) {
        executeDefenderLogic(homeBot2_, homeBot1_);
    }

    std::cerr << std::string(70, '=') << std::endl << std::endl;
}