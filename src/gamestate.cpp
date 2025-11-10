/**
 * @file GameState.cpp
 * @brief Implementation of game state manager and strategy
 * @author Agustin Valenzuela, Alex Petersen, Dylan Frigerio, Enzo Fernandez Rosas
 * @copyright Copyright (c) 2025
 */

#include "gamestate.h"

#include <algorithm>
#include <cmath>
#include <iostream>

#include "geometry.h"

// ============================================================================
// GAME STATE IMPLEMENTATION
// ============================================================================

GameState::GameState() : playerList_(PLAYER_COUNT), frameCount_(0) {
}

void GameState::reset() {
    frameCount_ = 0;
    playerList_[HOMEBOT_1].resetControls();
    playerList_[HOMEBOT_2].resetControls();
    currentRoles_ = RoleAssignment();
}

void GameState::parseFromJson(const json& message) {
    if (!message.contains("data")) {
        std::cerr << "Warning: Missing data field" << std::endl;
        return;
    }

    const json& data = message["data"];

    // Helper lambda to read robot/ball state
    auto readEntity = [&](const json& obj,
                          float& posX,
                          float& posY,
                          float& posZ,
                          float& rotX,
                          float& rotY,
                          float& rotZ,
                          float& velX,
                          float& velY,
                          float& velZ,
                          float& angVelX,
                          float& angVelY,
                          float& angVelZ) {
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

            if (obj.contains("angularVelocity") && obj["angularVelocity"].is_array() &&
                obj["angularVelocity"].size() >= 3) {
                angVelX = obj["angularVelocity"][0];
                angVelY = obj["angularVelocity"][1];
                angVelZ = obj["angularVelocity"][2];
            }
        } catch (const std::exception& e) {
            std::cerr << "Error reading entity: " << e.what() << std::endl;
        }
    };

    // Read robots using vector
    if (data.contains("homeBot1")) {
        float px, py, pz, rx, ry, rz, vx, vy, vz, avx, avy, avz;
        readEntity(data["homeBot1"], px, py, pz, rx, ry, rz, vx, vy, vz, avx, avy, avz);
        playerList_[HOMEBOT_1].setPosition(px, py, pz);
        playerList_[HOMEBOT_1].setRotation(rx, ry, rz);
        playerList_[HOMEBOT_1].setVelocity(vx, vy, vz);
        playerList_[HOMEBOT_1].setAngularVelocity(avx, avy, avz);
    }

    if (data.contains("homeBot2")) {
        float px, py, pz, rx, ry, rz, vx, vy, vz, avx, avy, avz;
        readEntity(data["homeBot2"], px, py, pz, rx, ry, rz, vx, vy, vz, avx, avy, avz);
        playerList_[HOMEBOT_2].setPosition(px, py, pz);
        playerList_[HOMEBOT_2].setRotation(rx, ry, rz);
        playerList_[HOMEBOT_2].setVelocity(vx, vy, vz);
        playerList_[HOMEBOT_2].setAngularVelocity(avx, avy, avz);
    }

    if (data.contains("rivalBot1")) {
        float px, py, pz, rx, ry, rz, vx, vy, vz, avx, avy, avz;
        readEntity(data["rivalBot1"], px, py, pz, rx, ry, rz, vx, vy, vz, avx, avy, avz);
        playerList_[RIVALBOT_1].setPosition(px, py, pz);
        playerList_[RIVALBOT_1].setRotation(rx, ry, rz);
        playerList_[RIVALBOT_1].setVelocity(vx, vy, vz);
        playerList_[RIVALBOT_1].setAngularVelocity(avx, avy, avz);
    }

    if (data.contains("rivalBot2")) {
        float px, py, pz, rx, ry, rz, vx, vy, vz, avx, avy, avz;
        readEntity(data["rivalBot2"], px, py, pz, rx, ry, rz, vx, vy, vz, avx, avy, avz);
        playerList_[RIVALBOT_2].setPosition(px, py, pz);
        playerList_[RIVALBOT_2].setRotation(rx, ry, rz);
        playerList_[RIVALBOT_2].setVelocity(vx, vy, vz);
        playerList_[RIVALBOT_2].setAngularVelocity(avx, avy, avz);
    }

    if (data.contains("ball")) {
        float px, py, pz, rx, ry, rz, vx, vy, vz, avx, avy, avz;
        readEntity(data["ball"], px, py, pz, rx, ry, rz, vx, vy, vz, avx, avy, avz);
        ball_.setPosition(px, py, pz);
        ball_.setRotation(rx, ry, rz);
        ball_.setVelocity(vx, vy, vz);
        ball_.setAngularVelocity(avx, avy, avz);
    }
}

json GameState::createCommandJson() const {
    json message = {
        {"type", "set"},
        {"data",
         {{"homeBot1",
           {{"positionXZ",
             {playerList_[HOMEBOT_1].getTargetX(), playerList_[HOMEBOT_1].getTargetZ()}},
            {"rotationY", playerList_[HOMEBOT_1].getTargetRotY()},
            {"dribbler", playerList_[HOMEBOT_1].getDribbler()},
            {"kick", playerList_[HOMEBOT_1].getKick()},
            {"chip", playerList_[HOMEBOT_1].getChip()}}},
          {"homeBot2",
           {{"positionXZ",
             {playerList_[HOMEBOT_2].getTargetX(), playerList_[HOMEBOT_2].getTargetZ()}},
            {"rotationY", playerList_[HOMEBOT_2].getTargetRotY()},
            {"dribbler", playerList_[HOMEBOT_2].getDribbler()},
            {"kick", playerList_[HOMEBOT_2].getKick()},
            {"chip", playerList_[HOMEBOT_2].getChip()}}}}}};
    return message;
}

// ============================================================================
// ROLE ASSIGNMENT
// ============================================================================

// STRATEGIES GameState::determineStrategy() {}

void GameState::assignRoles() {
    // Handle off-field robots
    bool bot1Active = playerList_[HOMEBOT_1].isOnField();
    bool bot2Active = playerList_[HOMEBOT_2].isOnField();

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
    float dist1 = playerList_[HOMEBOT_1].distanceTo(
        Robot(fieldMap_.predictedBallX, 0, fieldMap_.predictedBallZ));
    float dist2 = playerList_[HOMEBOT_2].distanceTo(
        Robot(fieldMap_.predictedBallX, 0, fieldMap_.predictedBallZ));

    // Check ball control
    bool bot1HasBall = playerList_[HOMEBOT_1].hasBallControl(ball_);
    bool bot2HasBall = playerList_[HOMEBOT_2].hasBallControl(ball_);

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
        striker.resetControls();
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
            bool blocked = fieldMap_.isShootingPathBlocked(
                striker, playerList_[RIVALBOT_1], playerList_[RIVALBOT_2]);
            if (blocked && distToGoal > 0.6f) {
                std::cerr << "  Using CHIP over obstacle" << std::endl;
                // Modify to chip shot (manual override)
            }
        }
        return;
    }

    // OPTION B: PASS if teammate better positioned
    if (defender.isOnField()) {
        float passQuality = fieldMap_.calculatePassQuality(
            striker, defender, playerList_[RIVALBOT_1], playerList_[RIVALBOT_2]);

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
    striker.dribbleToGoal();
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
            float passQuality = fieldMap_.calculatePassQuality(
                defender, striker, playerList_[RIVALBOT_1], playerList_[RIVALBOT_2]);

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
    playerList_[HOMEBOT_1].resetControls();
    playerList_[HOMEBOT_2].resetControls();

    std::cerr << "\n" << std::string(70, '=') << std::endl;
    std::cerr << "FRAME " << frameCount_ << " - Strategy Update" << std::endl;
    std::cerr << std::string(70, '=') << std::endl;

    // Update field map
    fieldMap_.update(ball_, playerList_[RIVALBOT_1], playerList_[RIVALBOT_2]);

    std::cerr << "\n[FIELD STATE]" << std::endl;
    std::cerr << "  Ball: (" << ball_.getPosX() << ", " << ball_.getPosZ() << ")" << std::endl;
    std::cerr << "  Ball predicted: (" << fieldMap_.predictedBallX << ", "
              << fieldMap_.predictedBallZ << ")" << std::endl;
    std::cerr << "  Dist to own goal: " << fieldMap_.ballDistToOwnGoal << "m" << std::endl;
    std::cerr << "  Active opponents: " << fieldMap_.activeOpponents << "/2" << std::endl;

    // Assign roles
    assignRoles();

    std::cerr << "\n[ROLES]" << std::endl;
    std::cerr << "  Bot1: " << (currentRoles_.bot1Role == Role::STRIKER ? "STRIKER" : "DEFENDER")
              << std::endl;
    std::cerr << "  Bot2: " << (currentRoles_.bot2Role == Role::STRIKER ? "STRIKER" : "DEFENDER")
              << std::endl;

    // Execute behaviors
    if (currentRoles_.bot1Role == Role::STRIKER) {
        executeStrikerLogic(playerList_[HOMEBOT_1], playerList_[HOMEBOT_2]);
    } else if (currentRoles_.bot1Role == Role::DEFENDER) {
        executeDefenderLogic(playerList_[HOMEBOT_1], playerList_[HOMEBOT_2]);
    }

    if (currentRoles_.bot2Role == Role::STRIKER) {
        executeStrikerLogic(playerList_[HOMEBOT_2], playerList_[HOMEBOT_1]);
    } else if (currentRoles_.bot2Role == Role::DEFENDER) {
        executeDefenderLogic(playerList_[HOMEBOT_2], playerList_[HOMEBOT_1]);
    }

    std::cerr << std::string(70, '=') << std::endl << std::endl;
}

// ========================================================================
// INTERACTIONS
// ========================================================================

Player GameState::whoHasBallControl() const {
    for (int i = 0; i < PLAYER_COUNT; ++i) {
        if (playerList_[i].hasBallControl(ball_)) {
            return (Player)i;  // Returns Player enum value
        }
    }
    return NONE;  // No one has control
}

int8_t GameState::quickPass() {
    static uint8_t state = 0;
    static float factor = 0.0f;
    Player passer = whoHasBallControl();
    if (passer == NONE || passer > HOMEBOT_2) {
        std::cerr << "No one has ball control, cannot pass." << std::endl;
        state = 0;
        factor = 0;
        return -1;
    }
    Player receiver = (passer == HOMEBOT_1) ? HOMEBOT_2 : HOMEBOT_1;

    if (!playerList_[receiver].isOnField()) {
        std::cerr << "Receiver is off field" << std::endl;
        state = 0;
        factor = 0;
        return -1;
    }

    // Simple pass implementation
    Robot& passerBot = playerList_[passer];
    Robot& receiverBot = playerList_[receiver];
    float angle;

    if (std::fabs((angle = normalizeAngle(
                       angleDifference(passerBot.getRotY(), receiverBot.getRotY()) - M_PI))) >
        0.11f) {
        if (passerBot.getAngVelY() < 0 && std::fabs(angle) < 0.4f && state == 0) {
            factor = -0.20f;
            state = 1;
        } else if (passerBot.getAngVelY() > 0 && std::fabs(angle) < 0.4f && state == 0) {
            factor = 0.16f;
            state = 1;
        }
        // Rotate towards receiver
        passerBot.setDribbler(1.0f);
        receiverBot.setDribbler(1.0f);
        passerBot.setTargetRotY(factor + smoothRotation(passerBot.getRotY(),
                                                        angleTo(passerBot.getPosX(),
                                                                passerBot.getPosZ(),
                                                                receiverBot.getPosX(),
                                                                receiverBot.getPosZ()),
                                                        0.60f));

        // passerBot.faceTowards(receiverBot.getPosX(), receiverBot.getPosZ());
        receiverBot.faceTowards(passerBot.getPosX(), passerBot.getPosZ());
        std::cerr << "passer angle" << passerBot.getRotY() << std::endl;
        std::cerr << "passer rotational velocity" << passerBot.getAngVelY() << std::endl;
        std::cerr << "receiver angle" << receiverBot.getRotY() << std::endl;
        std::cerr << "difference angle" << angle << std::endl;
        std::cerr << "Aligning for pass..." << std::endl;
        return 0;
    } else {
        // Calculate kick power based on distance
        float dist = passerBot.distanceTo(receiverBot);
        float kickPower = std::clamp(dist / MAX_PASS_DISTANCE, 0.1f, 0.8f);

        passerBot.setDribbler(0.0f);   // Disable dribbler for pass
        passerBot.setKick(kickPower);  // Disable dribbler for pass

        std::cerr << "Pass distance " << dist << std::endl;

        // Execute pass (this would need to be integrated into the control flow)
        std::cerr << "Quick pass from Bot" << (passer == HOMEBOT_1 ? "1" : "2") << " to Bot"
                  << (receiver == HOMEBOT_1 ? "1" : "2") << " (power: " << kickPower << ")"
                  << std::endl;
        state = 0;
        factor = 0;
        return 1;
    }
}

int8_t GameState::quickChip() {
    static uint8_t state = 0;
    static float factor = 0.0f;
    Player passer = whoHasBallControl();
    if (passer == NONE || passer > HOMEBOT_2) {
        std::cerr << "No one has ball control, cannot pass." << std::endl;
        state = 0;
        factor = 0;
        return -1;
    }
    Player receiver = (passer == HOMEBOT_1) ? HOMEBOT_2 : HOMEBOT_1;

    if (!playerList_[receiver].isOnField()) {
        std::cerr << "Receiver is off field" << std::endl;
        state = 0;
        factor = 0;
        return -1;
    }

    // Simple pass implementation
    Robot& passerBot = playerList_[passer];
    Robot& receiverBot = playerList_[receiver];
    float angle;

    if (std::fabs((angle = normalizeAngle(
                       angleDifference(passerBot.getRotY(), receiverBot.getRotY()) - M_PI))) >
        0.11f) {
        if (passerBot.getAngVelY() < 0 && std::fabs(angle) < 0.4f && state == 0) {
            factor = -0.2f;
            state = 1;
        } else if (passerBot.getAngVelY() > 0 && std::fabs(angle) < 0.4f && state == 0) {
            factor = 0.2f;
            state = 1;
        }
        // Rotate towards receiver
        passerBot.setDribbler(1.0f);
        receiverBot.setDribbler(1.0f);
        passerBot.setTargetRotY(factor + smoothRotation(passerBot.getRotY(),
                                                        angleTo(passerBot.getPosX(),
                                                                passerBot.getPosZ(),
                                                                receiverBot.getPosX(),
                                                                receiverBot.getPosZ()),
                                                        0.60f));

        // passerBot.faceTowards(receiverBot.getPosX(), receiverBot.getPosZ());
        receiverBot.faceTowards(passerBot.getPosX(), passerBot.getPosZ());
        std::cerr << "passer angle" << passerBot.getRotY() << std::endl;
        std::cerr << "passer rotational velocity" << passerBot.getAngVelY() << std::endl;
        std::cerr << "receiver angle" << receiverBot.getRotY() << std::endl;
        std::cerr << "difference angle" << angle << std::endl;
        std::cerr << "Aligning for pass..." << std::endl;
        return 0;
    } else {
        // Calculate kick power based on distance
        float dist = passerBot.distanceTo(receiverBot);
        float kickPower = std::clamp(0.9f * dist / MAX_PASS_DISTANCE, 0.1f, 0.8f);

        passerBot.setChip(0.0f);       // Disable dribbler for pass
        passerBot.setKick(kickPower);  // Disable dribbler for pass

        std::cerr << "Pass distance " << dist << std::endl;

        // Execute pass (this would need to be integrated into the control flow)
        std::cerr << "Quick pass from Bot" << (passer == HOMEBOT_1 ? "1" : "2") << " to Bot"
                  << (receiver == HOMEBOT_1 ? "1" : "2") << " (power: " << kickPower << ")"
                  << std::endl;
        state = 0;
        factor = 0;
        return 1;
    }
}
