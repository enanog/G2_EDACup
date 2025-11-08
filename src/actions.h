// ============================================================================
// actions.h
// Low-level robot action primitives
// ============================================================================

#ifndef ACTIONS_H
#define ACTIONS_H

#include "game_state.h"
#include <nlohmann/json.hpp>

using json = nlohmann::json;

/**
 * Command a robot to move toward a target position
 * Handles smooth movement and rotation
 */
void moveTo(const Robot& robot, Vec2 target, double targetRotation, json& command);

/**
 * Command a robot to face toward a target point
 * Updates rotation smoothly without moving
 */
void faceTarget(const Robot& robot, Vec2 target, json& command);

/**
 * Command a robot to stop at current position
 */
void stop(const Robot& robot, json& command);

/**
 * Command a robot to dribble the ball (activates dribbler)
 */
void dribble(const Robot& robot, Vec2 target, double rotation, double power, json& command);

/**
 * Command a robot to kick the ball
 */
void kick(const Robot& robot, Vec2 target, double rotation, double power, json& command);

/**
 * Command a robot to chip the ball (lofted kick)
 */
void chip(const Robot& robot, Vec2 target, double rotation, double power, json& command);

/**
 * Set basic movement command without special actions
 */
void setBasicMovement(Vec2 position, double rotation, json& command);

/**
 * Check if robot should back off due to holding violation
 */
bool shouldBackOff(const Robot& robot);

#endif // ACTIONS_H
