/**
 * @file strategy.h
 * @brief Game strategy and decision-making
 * @author Agustin Valenzuela,
 *         Alex Petersen,
 *         Dylan Frigerio,
 *         Enzo Fernandez Rosas
 *
 * @copyright Copyright (c) 2025
 */

#ifndef STRATEGY_H
#define STRATEGY_H

#include <cstdint>

#include "robot.h"

 // Complete game state with centralized command storage
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

// Main strategy function - updates commands in GameState
void decideStrategy(GameState& state);

#endif // STRATEGY_H