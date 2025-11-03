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
 // SIMPLE ROLE SYSTEM
 // ============================================================================

enum class Role {
    ATTACKER,   // Controls the ball and attacks
    DEFENDER    // Defends and supports
};

struct RoleConfig {
    Role role;
    float targetX, targetZ;
};

class RoleManager {
private:
    RoleConfig bot1Config;
    RoleConfig bot2Config;

public:
    RoleManager() {
        bot1Config.role = Role::ATTACKER;
        bot2Config.role = Role::DEFENDER;
    }

    void updateRoles(const GameState& state) {
        // Assign attacker to robot closest to ball
        float dist1 = distance(state.homeBot1.posX, state.homeBot1.posZ,
            state.ball.posX, state.ball.posZ);
        float dist2 = distance(state.homeBot2.posX, state.homeBot2.posZ,
            state.ball.posX, state.ball.posZ);

        if (dist1 < dist2) {
            bot1Config.role = Role::ATTACKER;
            bot2Config.role = Role::DEFENDER;
        }
        else {
            bot1Config.role = Role::DEFENDER;
            bot2Config.role = Role::ATTACKER;
        }

        // Calculate defender position
        calculateDefenderPosition(state);
    }

    void calculateDefenderPosition(const GameState& state) {
        // Defender positions between ball and our goal
        float defX = (state.ball.posX + LEFT_GOAL_X) * 0.6f;
        float defZ = state.ball.posZ * 0.5f;

        // Keep defender in valid field position
        defX = std::max(defX, LEFT_GOAL_X + 0.3f);
        defX = std::clamp(defX, -FIELD_HALF_LENGTH + 0.2f, -0.2f);
        defZ = std::clamp(defZ, -FIELD_HALF_WIDTH + 0.2f, FIELD_HALF_WIDTH - 0.2f);

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

// ============================================================================
// GLOBAL ROLE MANAGER
// ============================================================================

static RoleManager roleManager;

// ============================================================================
// DECISION MAKING FUNCTIONS
// ============================================================================

RobotCommand decideAttackerAction(const RobotState& robot, const RobotState& ball) {
    std::cerr << ">> Attacker Decision <<" << std::endl;

    // If no ball control, go get it
    if (!hasBallControl(robot, ball)) {
        float distToBall = distance(robot.posX, robot.posZ, ball.posX, ball.posZ);
        if (distToBall > 0.5f) {
            std::cerr << "Chasing ball" << std::endl;
            return BallControl::chaseBall(robot, ball);
        }
        else {
            std::cerr << "Intercepting ball" << std::endl;
            return BallControl::interceptBall(robot, ball);
        }
    }

    std::cerr << "Has ball control" << std::endl;
    
    // Shoot if close to opponent goal
    float distToGoal = distance(ball.posX, ball.posZ, RIGHT_GOAL_X, 0.0f);
    if (distToGoal < 1.5f) {
        auto shootResult = Shooting::shootAtGoal(robot, ball);
        if (shootResult.success) {
            std::cerr << ">>> SHOOTING AT GOAL! <<<" << std::endl;
            return shootResult.command;
        }
    }
    
    // Otherwise dribble toward goal
    std::cerr << "Dribbling toward goal" << std::endl;
    return BallControl::dribbleTo(robot, ball, RIGHT_GOAL_X - 0.3f, 0.0f);
}

RobotCommand decideDefenderAction(const RobotState& robot, const RobotState& ball,
    const RoleConfig& config) {
    std::cerr << ">> Defender Decision <<" << std::endl;

    // Clear ball if we have control and it's close
    if (hasBallControl(robot, ball)) {
        std::cerr << "Defender clearing ball" << std::endl;
        return Shooting::clearBall(robot, ball);
    }

    // Otherwise maintain defensive position
    std::cerr << "Maintaining defensive position" << std::endl;
    return Positioning::moveTo(robot, config.targetX, config.targetZ);
}

// ============================================================================
// MAIN STRATEGY FUNCTION
// ============================================================================

void decideStrategy(const GameState& state, RobotCommand& bot1Cmd, RobotCommand& bot2Cmd) {
    // Update roles based on current game state
    roleManager.updateRoles(state);

    const RoleConfig& bot1Config = roleManager.getBot1Config();
    const RoleConfig& bot2Config = roleManager.getBot2Config();

    // Assign actions based on roles
    if (bot1Config.role == Role::ATTACKER) {
        bot1Cmd = decideAttackerAction(state.homeBot1, state.ball);
    }
    else {
        bot1Cmd = decideDefenderAction(state.homeBot1, state.ball, bot1Config);
    }

    if (bot2Config.role == Role::ATTACKER) {
        bot2Cmd = decideAttackerAction(state.homeBot2, state.ball);
    }
    else {
        bot2Cmd = decideDefenderAction(state.homeBot2, state.ball, bot2Config);
    }
}