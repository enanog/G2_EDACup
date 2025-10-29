/**
 * @file strategy.cpp
 * @brief Implementation of game strategy and decision-making
 * @author Agustin Valenzuela,
 *         Alex Petersen,
 *         Dylan Frigerio,
 *         Enzo Fernandez Rosas
 *
 * @copyright Copyright (c) 2025
 */

#include "strategy.h"
#include "robot.h"
#include "geometry.h"
#include "constants.h"
#include <cmath>
#include <iostream>

void decideStrategy(const GameState& state, RobotCommand& bot1Cmd, RobotCommand& bot2Cmd)
{
	// Verificar que la pelota esté dentro del campo
	if (isOutOfBounds(state.ball.posX, state.ball.posZ)) {
		std::cerr << "WARNING: Ball out of bounds at ("
			<< state.ball.posX << ", " << state.ball.posZ << ")" << std::endl;
	}

	// Robot 1: seguir la pelota
	std::cerr << "Bot1 moving to ball at ("
		<< state.ball.posX << ", " << state.ball.posZ << ")" << std::endl;

	bot1Cmd = moveToPosition(state.homeBot1, state.ball.posX, state.ball.posZ);
}