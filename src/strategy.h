// ============================================================================
// strategy.h
// High-level strategy and decision-making
// ============================================================================

#ifndef STRATEGY_H
#define STRATEGY_H

#include "game_state.h"
#include <nlohmann/json.hpp>

using json = nlohmann::json;

/**
 * Execute attacker (homeBot1) strategy
 * Chases ball, dribbles toward goal, shoots when aligned
 */
void executeAttacker(const GameState& state, json& command);

/**
 * Execute defender (homeBot2) strategy
 * Protects own goal, tracks ball, clears dangerous situations
 */
void executeDefender(const GameState& state, json& command);

/**
 * Helper: Calculate best shooting angle and check if aligned
 */
bool isAlignedForShot(const Robot& robot, Vec2 ballPos, Vec2 goalPos, double& angleToGoal);

/**
 * Helper: Determine if attacker should shoot now
 */
bool shouldShoot(const Robot& robot, const Ball& ball, double alignmentAngle);

/**
 * Helper: Get defender home position based on ball location
 */
Vec2 getDefenderPosition(const Ball& ball);

#endif // STRATEGY_H
