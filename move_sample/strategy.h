/**
 * @file strategy.h
 * @brief Game strategy and decision-making functions
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

 /**
  * @brief Represents the complete game state
  */
struct GameState {
	RobotState homeBot1;     // First home robot
	RobotState homeBot2;     // Second home robot
	RobotState rivalBot1;    // First rival robot
	RobotState rivalBot2;    // Second rival robot
	RobotState ball;         // Ball state
};

/**
 * @brief Main strategy function - decides actions for both robots
 * @param state Current game state
 * @param bot1Cmd Output command for robot 1
 * @param bot2Cmd Output command for robot 2
 */
void decideStrategy(const GameState& state, RobotCommand& bot1Cmd, RobotCommand& bot2Cmd);

#endif // STRATEGY_H