/**
 * @file strategy.h
 * @brief Game strategy and decision-making - EDACup 2025
 * @author Agustin Valenzuela, Alex Petersen, Dylan Frigerio, Enzo Fernandez Rosas
 * @copyright Copyright (c) 2025
 */

#ifndef STRATEGY_H
#define STRATEGY_H

#include <cstdint>
#include "robot.h"

 // ============================================================================
 // ROLE SYSTEM
 // ============================================================================

 /**
  * @brief Robot roles in dynamic strategy
  * STRIKER: Attack, shoot, pass forward
  * DEFENDER: Protect goal, mark threats, support
  */
enum class Role {
    STRIKER,
    DEFENDER,
    OFFFIELD
};

/**
 * @brief Role assignment for both robots
 */
struct RoleAssignment {
    Role bot1Role;
    Role bot2Role;

    RoleAssignment() : bot1Role(Role::STRIKER), bot2Role(Role::DEFENDER) {}
    RoleAssignment(Role b1, Role b2) : bot1Role(b1), bot2Role(b2) {}
};

// ============================================================================
// GAME STATE
// ============================================================================

/**
 * @brief Complete game state with centralized command storage
 */
struct GameState {
    // Robot and ball states (read from simulator)
    RobotState homeBot1;     // First home robot
    RobotState homeBot2;     // Second home robot
    RobotState rivalBot1;    // First rival robot
    RobotState rivalBot2;    // Second rival robot
    RobotState ball;         // Ball state

    // Commands to be sent (written by strategy)
    RobotCommand bot1Cmd;    // Command for homeBot1
    RobotCommand bot2Cmd;    // Command for homeBot2

    // Frame counter
    uint32_t frameCount;

    // Constructor
    GameState() : frameCount(0) {}

    // Reset commands before new frame
    void resetCommands() {
        bot1Cmd.reset();
        bot2Cmd.reset();
    }
};

// ============================================================================
// MAIN STRATEGY FUNCTION
// ============================================================================

/**
 * @brief Main strategy decision-making function
 * Updates bot1Cmd and bot2Cmd in GameState
 */
void decideStrategy(GameState& state);

#endif // STRATEGY_H