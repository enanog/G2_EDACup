// ============================================================================
// utils.h
// Utility functions for boundary checking, logging, and game rules
// ============================================================================

#ifndef UTILS_H
#define UTILS_H

#include "game_state.h"
#include <string>

// Field and game constants
// IMPORTANT: Field orientation in EDACup simulator:
// - Z axis: Field LENGTH (goals at Z = ±1.095)
// - X axis: Field WIDTH (sidelines at X = ±0.79)
constexpr double FIELD_HALF_LENGTH = 1.095;  // Half of field length in Z axis (m)
constexpr double FIELD_HALF_WIDTH = 0.79;    // Half of field width in X axis (m)
constexpr double SAFE_WALL_DISTANCE = 0.12;  // Stay away from walls (m)

// Goal positions (goals are at Z extremes)
constexpr double FORWARD_GOAL_Z = FIELD_HALF_LENGTH;   // Opponent goal at +Z
constexpr double BACKWARD_GOAL_Z = -FIELD_HALF_LENGTH; // Own goal at -Z

// Penalty area dimensions
constexpr double PENALTY_DEPTH = 0.15;   // Distance from goal line
constexpr double PENALTY_WIDTH = 0.35;   // Half-width of penalty box

// Rule parameters
constexpr double BALL_HOLD_TIME_LIMIT = 10.0;     // Max seconds holding ball
constexpr double BALL_STATIONARY_LIMIT = 5.0;     // Max seconds ball stationary
constexpr int TIMEOUT_DURATION_FRAMES = 1200;     // 60 seconds at 50ms/frame
constexpr double FRAME_TIME = 0.05;               // 50ms per frame

// Physics and control parameters
constexpr double BALL_TOUCH_DISTANCE = 0.10;      // Distance to consider touching ball
constexpr double BALL_STATIONARY_SPEED = 0.05;    // Speed threshold for stationary
constexpr double SMOOTH_ROTATION_SPEED = 0.15;    // Max rotation per frame (rad)

/**
 * Check if position is out of field bounds
 */
bool isOutOfBounds(Vec2 pos);

/**
 * Check if position is inside a penalty area
 * forwardSide: true for opponent goal area (+Z), false for own goal area (-Z)
 */
bool isInPenaltyArea(Vec2 pos, bool forwardSide);

/**
 * Clamp position to stay within safe field boundaries
 */
Vec2 clampToField(Vec2 pos);

/**
 * Get safe target position that avoids boundaries and penalty areas
 */
Vec2 getSafeTarget(Vec2 desired, Vec2 currentPos);

/**
 * Check if robot is touching the ball
 */
bool isTouchingBall(Vec2 robotPos, Vec2 ballPos);

/**
 * Check if ball is effectively stationary
 */
bool isBallStationary(Vec2 ballVel);

/**
 * Update rule enforcement tracking for a robot
 */
void updateRobotRules(Robot& robot, const Ball& ball);

/**
 * Update ball tracking (stationarity, etc.)
 */
void updateBallTracking(Ball& ball);

/**
 * Log debug message with timestamp
 */
void logDebug(const std::string& message);

/**
 * Log robot state for debugging
 */
void logRobotState(const std::string& name, const Robot& robot);

#endif // UTILS_H