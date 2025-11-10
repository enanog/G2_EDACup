/**
 * @file gamestate.cpp
 * @brief Global game state manager implementation
 * @author Agustin Valenzuela, Alex Petersen, Dylan Frigerio, Enzo Fernandez Rosas
 * @copyright Copyright (c) 2025
 */

#include "gamestate.h"

#include <algorithm>
#include <cmath>
#include <iostream>

#include "ball.h"
#include "constants.h"
#include "field.h"
#include "geometry.h"
#include "robot.h"
#include "strategy.h"

// ============================================================================
// CONSTRUCTOR & INITIALIZATION
// ============================================================================

GameState::GameState()
    : playerList_(PLAYER_COUNT), ball_(), strategyManager_(), fieldMap_(), frameCount_(0) {
}

void GameState::reset() {
    frameCount_ = 0;
    playerList_[HOMEBOT_1].resetControls();
    playerList_[HOMEBOT_2].resetControls();
}

// ============================================================================
// JSON PARSING
// ============================================================================

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
// STRATEGY EXECUTION
// ============================================================================

void GameState::update() {
    // Reset robot controls at start of each frame
    playerList_[HOMEBOT_1].resetControls();
    playerList_[HOMEBOT_2].resetControls();

    // Update field map with current game state
    fieldMap_.update(ball_, playerList_[RIVALBOT_1], playerList_[RIVALBOT_2]);

    // Delegate strategy execution to StrategyManager
    strategyManager_.update(*this);
}

// ============================================================================
// BALL CONTROL DETECTION
// ============================================================================

Player GameState::whoHasBallControl() const {
    if (playerList_[HOMEBOT_1].hasBallControl(ball_)) {
        return HOMEBOT_1;
    }
    if (playerList_[HOMEBOT_2].hasBallControl(ball_)) {
        return HOMEBOT_2;
    }
    if (playerList_[RIVALBOT_1].hasBallControl(ball_)) {
        return RIVALBOT_1;
    }
    if (playerList_[RIVALBOT_2].hasBallControl(ball_)) {
        return RIVALBOT_2;
    }
    return NONE;
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

Robot& receiverBot = playerList_[receiver];
float angle;

if (std::fabs((angle = normalizeAngle(angleDifference(passerBot.getRotY(), receiverBot.getRotY()) - M_PI))) > 0.11f) {
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
