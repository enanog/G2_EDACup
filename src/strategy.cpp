/**
 * @file strategy.cpp
 * @brief Main strategy implementation
 * @author Agustin Valenzuela,
 *         Alex Petersen,
 *         Dylan Frigerio,
 *         Enzo Fernandez Rosas
 *
 * @copyright Copyright (c) 2025
 */

#include "strategy.h"
#include "actions.h"
#include "geometry.h"
#include "constants.h"
#include <iostream>
#include <algorithm>

 // ============================================================================
 // ROLE SYSTEM
 // ============================================================================

enum class Role {
    ATTACKER,   // Controls the ball and attacks
    DEFENDER    // Defends and supports
};

struct RoleConfig {
    Role role;
    float targetX, targetZ;  // Defensive position
};

class RoleManager {
private:
    RoleConfig bot1Config;
    RoleConfig bot2Config;

public:
    RoleManager() {
        // Default: bot1 attacks, bot2 defends
        bot1Config.role = Role::ATTACKER;
        bot2Config.role = Role::DEFENDER;
        bot1Config.targetX = 0.0f;
        bot1Config.targetZ = 0.0f;
        bot2Config.targetX = -0.7f;
        bot2Config.targetZ = 0.0f;
    }

    void updateRoles(const GameState& state) {
        // Assign attacker to robot closest to ball
        float dist1 = distance(state.homeBot1.posX, state.homeBot1.posZ,
            state.ball.posX, state.ball.posZ);
        float dist2 = distance(state.homeBot2.posX, state.homeBot2.posZ,
            state.ball.posX, state.ball.posZ);

        // Update roles based on distance (closest becomes attacker)
        if (dist1 < dist2) {
            bot1Config.role = Role::ATTACKER;
            bot2Config.role = Role::DEFENDER;
        }
        else {
            bot1Config.role = Role::DEFENDER;
            bot2Config.role = Role::ATTACKER;
        }

        // Update defender position
        calculateDefenderPosition(state);
    }

    void calculateDefenderPosition(const GameState& state) {
        // Defender positions between ball and our goal
        float defX = (state.ball.posX + LEFT_GOAL_X) * 0.6f;
        float defZ = state.ball.posZ * 0.5f;  // Track ball laterally

        // Keep defender in safe position
        defX = std::max(defX, LEFT_GOAL_X + 0.3f);
        defX = std::clamp(defX, -FIELD_HALF_LENGTH + 0.2f, -0.2f);
        defZ = std::clamp(defZ, -FIELD_HALF_WIDTH + 0.2f, FIELD_HALF_WIDTH - 0.2f);

        // Update appropriate config
        if (bot1Config.role == Role::DEFENDER) {
            bot1Config.targetX = defX;
            bot1Config.targetZ = defZ;
        }
        else {
            bot2Config.targetX = defX;
            bot2Config.targetZ = defZ;
        }
    }

    const RoleConfig& getBot1Config() const { return bot1Config; }
    const RoleConfig& getBot2Config() const { return bot2Config; }
};

// Global role manager
static RoleManager roleManager;

// ============================================================================
// DECISION FUNCTIONS
// ============================================================================

/**
 * Attacker decision logic
 */
void decideAttackerAction(const RobotState& robot, const RobotState& ball,
    RobotCommand& cmd) {
    std::cerr << "\n>> ATTACKER LOGIC <<" << std::endl;

    // Calculate distance to ball and goal
    float distToBall = distance(robot.posX, robot.posZ, ball.posX, ball.posZ);
    float distToGoal = distance(ball.posX, ball.posZ, RIGHT_GOAL_X, 0.0f);

    std::cerr << "  Dist to ball: " << distToBall
        << "m, Dist to goal: " << distToGoal << "m" << std::endl;

    // STATE 1: Don't have ball - go get it
    if (!hasBallControl(robot, ball)) {
        if (distToBall > 0.5f) {
            BallControl::chaseBall(robot, ball, cmd);
        }
        else {
            BallControl::interceptBall(robot, ball, cmd);
        }
        return;
    }

    std::cerr << "  ✓ Has ball control" << std::endl;

    // STATE 2: Have ball - decide between shoot or dribble
    if (distToGoal < 1.5f) {
        // Close enough to attempt shot
        ActionResult shootResult = Shooting::shootAtGoal(robot, ball, cmd);
        if (shootResult.success) {
            std::cerr << "  >>> TAKING SHOT! <<<" << std::endl;
        }
    }
    else {
        // Too far - dribble toward goal
        BallControl::dribbleTo(robot, ball, RIGHT_GOAL_X - 0.3f, 0.0f, cmd);
    }
}

/**
 * Defender decision logic
 */
void decideDefenderAction(const RobotState& robot, const RobotState& ball,
    const RoleConfig& config, RobotCommand& cmd) {
    std::cerr << "\n>> DEFENDER LOGIC <<" << std::endl;

    // Calculate distances
    float distToBall = distance(robot.posX, robot.posZ, ball.posX, ball.posZ);
    float ballDistToOwnGoal = distance(ball.posX, ball.posZ, LEFT_GOAL_X, 0.0f);

    std::cerr << "  Ball dist to own goal: " << ballDistToOwnGoal << "m" << std::endl;

    // STATE 1: Ball very close to our goal and we can reach it - clear it!
    if (ballDistToOwnGoal < 0.6f && distToBall < 0.3f) {
        std::cerr << "  ⚠ DANGER! Clearing ball!" << std::endl;
        Shooting::clearBall(robot, ball, cmd);
        return;
    }

    // STATE 2: Have ball control - clear it away
    if (hasBallControl(robot, ball)) {
        std::cerr << "  Defender has ball - clearing" << std::endl;
        Shooting::clearBall(robot, ball, cmd);
        return;
    }

    // STATE 3: Normal defensive positioning
    std::cerr << "  Maintaining defensive position ("
        << config.targetX << ", " << config.targetZ << ")" << std::endl;
    Positioning::moveTo(robot, config.targetX, config.targetZ, cmd);

    // Face the ball while defending
    cmd.targetRotY = smoothRotation(robot.rotY,
        angleTo(robot.posX, robot.posZ, ball.posX, ball.posZ),
        0.15f);
}

// ============================================================================
// MAIN STRATEGY FUNCTION
// ============================================================================

/**
 * Main strategy - decides actions for both robots
 * Modifies commands directly in GameState
 */
void decideStrategy(GameState& state) {
    // Reset commands before deciding
    state.resetCommands();

    // Update roles based on current situation
    roleManager.updateRoles(state);

    const RoleConfig& bot1Config = roleManager.getBot1Config();
    const RoleConfig& bot2Config = roleManager.getBot2Config();

    // Print roles
    std::cerr << "\n========== FRAME " << state.frameCount << " ==========" << std::endl;
    std::cerr << "Ball position: (" << state.ball.posX << ", " << state.ball.posZ << ")" << std::endl;
    std::cerr << "Bot1 role: " << (bot1Config.role == Role::ATTACKER ? "ATTACKER" : "DEFENDER") << std::endl;
    std::cerr << "Bot2 role: " << (bot2Config.role == Role::ATTACKER ? "ATTACKER" : "DEFENDER") << std::endl;

    // Execute strategy based on roles
    if (bot1Config.role == Role::ATTACKER) {
        decideAttackerAction(state.homeBot1, state.ball, state.bot1Cmd);
    }
    else {
        decideDefenderAction(state.homeBot1, state.ball, bot1Config, state.bot1Cmd);
    }

    if (bot2Config.role == Role::ATTACKER) {
        decideAttackerAction(state.homeBot2, state.ball, state.bot2Cmd);
    }
    else {
        decideDefenderAction(state.homeBot2, state.ball, bot2Config, state.bot2Cmd);
    }

    std::cerr << "==============================\n" << std::endl;
}