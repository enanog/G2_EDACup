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

#include "robot.h"

 // Complete game state
struct GameState {
    RobotState homeBot1;     // First home robot
    RobotState homeBot2;     // Second home robot
    RobotState rivalBot1;    // First rival robot
    RobotState rivalBot2;    // Second rival robot
    RobotState ball;         // Ball state
};

// Main strategy function
void decideStrategy(const GameState& state, RobotCommand& bot1Cmd, RobotCommand& bot2Cmd);

#endif // STRATEGY_H