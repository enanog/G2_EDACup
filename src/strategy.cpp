/**
 * @file strategy.cpp
 * @brief Main strategy implementation - SIMPLIFIED (Attacker/Defender)
 * @author Agustin Valenzuela, Alex Petersen, Dylan Frigerio, Enzo Fernandez Rosas
 * @copyright Copyright (c) 2025
 */

#include "strategy.h"
#include "actions.h"
#include "geometry.h"
#include "constants.h"
#include <iostream>
#include <algorithm>

 // ============================================================================
 // SIMPLE ROLE SYSTEM - ATTACKER AND DEFENDER ONLY
 // ============================================================================

enum class Role {
    ATTACKER,   // Always attacks and shoots
    DEFENDER    // Always defends our goal
};

/**
 * Attacker decision logic - SIMPLIFIED
 */
void decideAttackerAction(const RobotState& robot, const RobotState& ball,
    const GameState& state, RobotCommand& cmd) {
    std::cerr << "\n>> ATTACKER LOGIC <<" << std::endl;

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

    // STATE 2: Have ball - try to shoot if close enough
    if (distToGoal < 1.5f) {
        ActionResult shootResult = Shooting::shootAtGoal(robot, ball, cmd);
        if (shootResult.success) {
            std::cerr << "  >>> TAKING SHOT! <<<" << std::endl;
            return;
        }
        // If positioning for shot, continue with that
        if (distToGoal < 1.0f) {
            return;
        }
    }

    // STATE 3: Dribble toward goal slowly
    std::cerr << "  Dribbling toward goal with controlled rotation..." << std::endl;
    BallControl::dribbleToGoal(robot, ball, cmd);
}

/**
 * Defender decision logic - SIMPLIFIED
 */
void decideDefenderAction(const RobotState& robot, const RobotState& ball,
    const GameState& state, RobotCommand& cmd) {
    std::cerr << "\n>> DEFENDER LOGIC <<" << std::endl;

    float distToBall = distance(robot.posX, robot.posZ, ball.posX, ball.posZ);
    float ballDistToOwnGoal = distance(ball.posX, ball.posZ, LEFT_GOAL_X, 0.0f);

    std::cerr << "  Ball dist to own goal: " << ballDistToOwnGoal << "m" << std::endl;

    // STATE 1: DANGER - Ball very close to our goal
    if (ballDistToOwnGoal < 0.7f) {
        std::cerr << "  ⚠ DANGER! Emergency defense!" << std::endl;

        if (distToBall < 0.4f) {
            // Close enough to clear it
            Shooting::clearBall(robot, ball, cmd);
        }
        else {
            // Chase the ball
            BallControl::chaseBall(robot, ball, cmd);
        }
        return;
    }

    // STATE 2: Have ball control - clear it away
    if (hasBallControl(robot, ball)) {
        std::cerr << "  Defender has ball - clearing" << std::endl;
        Shooting::clearBall(robot, ball, cmd);
        return;
    }

    // STATE 3: Normal defensive positioning
    // Position between ball and our goal
    float defX = (ball.posX + LEFT_GOAL_X) * 0.6f;
    float defZ = ball.posZ * 0.4f;  // Track ball laterally

    // Keep defender in safe defensive zone (not too far back)
    defX = std::max(defX, LEFT_GOAL_X + 0.35f);  // Stay out of goal area
    defX = std::clamp(defX, -FIELD_HALF_LENGTH + 0.3f, -0.3f);
    defZ = std::clamp(defZ, -FIELD_HALF_WIDTH + 0.3f, FIELD_HALF_WIDTH - 0.3f);

    std::cerr << "  Maintaining defensive position ("
        << defX << ", " << defZ << ")" << std::endl;

    // Move to position while always facing the ball
    Positioning::moveToWhileFacing(robot, defX, defZ,
        ball.posX, ball.posZ, cmd);
}

// ============================================================================
// MAIN STRATEGY FUNCTION - SIMPLIFIED
// ============================================================================

/**
 * Simple strategy: Bot1 always attacks, Bot2 always defends
 */
void decideStrategy(GameState& state) {
    state.resetCommands();

    std::cerr << "\n========== FRAME " << state.frameCount << " ==========" << std::endl;
    std::cerr << "Ball: (" << state.ball.posX << ", " << state.ball.posZ
        << ") vel: (" << state.ball.velX << ", " << state.ball.velZ << ")" << std::endl;
    std::cerr << "Bot1: ATTACKER at (" << state.homeBot1.posX << ", "
        << state.homeBot1.posZ << ")" << std::endl;
    std::cerr << "Bot2: DEFENDER at (" << state.homeBot2.posX << ", "
        << state.homeBot2.posZ << ")" << std::endl;

    // Bot1 is always the ATTACKER
    decideAttackerAction(state.homeBot1, state.ball, state, state.bot1Cmd);

    // Bot2 is always the DEFENDER
    decideDefenderAction(state.homeBot2, state.ball, state, state.bot2Cmd);

    std::cerr << "==============================\n" << std::endl;
}