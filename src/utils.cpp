// ============================================================================
// utils.cpp
// Implementation of utility functions
// ============================================================================

#include "utils.h"
#include <iostream>
#include <iomanip>
#include <cmath>

using namespace std;

/**
 * Check if position exceeds safe field boundaries
 * Field: X = ±0.79m (width), Z = ±1.095m (length)
 */
bool isOutOfBounds(Vec2 pos) {
    double maxX = FIELD_HALF_WIDTH - SAFE_WALL_DISTANCE;   // X boundaries (sidelines)
    double maxZ = FIELD_HALF_LENGTH - SAFE_WALL_DISTANCE;  // Z boundaries (goal lines)

    return std::abs(pos.x) > maxX || std::abs(pos.z) > maxZ;
}

/**
 * Check if position is inside penalty area
 * forwardSide = true: check opponent's penalty area (forward, +Z)
 * forwardSide = false: check own penalty area (backward, -Z)
 */
bool isInPenaltyArea(Vec2 pos, bool forwardSide) {
    if (forwardSide) {
        // Forward penalty area (opponent goal at +Z)
        return pos.z > (FORWARD_GOAL_Z - PENALTY_DEPTH) &&
            std::abs(pos.x) < PENALTY_WIDTH;
    }
    else {
        // Backward penalty area (own goal at -Z)
        return pos.z < (BACKWARD_GOAL_Z + PENALTY_DEPTH) &&
            std::abs(pos.x) < PENALTY_WIDTH;
    }
}

/**
 * Constrain position to remain within field boundaries
 */
Vec2 clampToField(Vec2 pos) {
    double maxX = FIELD_HALF_WIDTH - SAFE_WALL_DISTANCE;   // X limits
    double maxZ = FIELD_HALF_LENGTH - SAFE_WALL_DISTANCE;  // Z limits

    pos.x = clamp(pos.x, -maxX, maxX);
    pos.z = clamp(pos.z, -maxZ, maxZ);

    return pos;
}

/**
 * Calculate a safe target position avoiding boundaries and penalty areas
 */
Vec2 getSafeTarget(Vec2 desired, Vec2 currentPos) {
    Vec2 target = desired;

    // First, clamp to field boundaries
    target = clampToField(target);

    // Avoid entering opponent penalty area completely
    if (isInPenaltyArea(target, true)) {
        // Push back out of penalty area
        target.z = FORWARD_GOAL_Z - PENALTY_DEPTH - 0.05;
    }

    return target;
}

/**
 * Determine if robot is close enough to be touching the ball
 */
bool isTouchingBall(Vec2 robotPos, Vec2 ballPos) {
    return distance(robotPos, ballPos) < BALL_TOUCH_DISTANCE;
}

/**
 * Determine if ball velocity is below stationary threshold
 */
bool isBallStationary(Vec2 ballVel) {
    return ballVel.length() < BALL_STATIONARY_SPEED;
}

/**
 * Update rule enforcement tracking for a robot each frame
 * Handles: timeouts, ball holding time, boundary violations
 */
void updateRobotRules(Robot& robot, const Ball& ball) {
    // Decrement timeout counter if in timeout
    if (robot.timeoutFrames > 0) {
        robot.timeoutFrames--;
        return;
    }

    // Track continuous ball contact time (anti-holding rule)
    bool touching = isTouchingBall(robot.pos, ball.pos);

    if (touching) {
        if (robot.wasTouchingBall) {
            // Continue counting touch time
            robot.ballTouchTime += FRAME_TIME;
        }
        else {
            // Just started touching
            robot.ballTouchTime = 0;
        }
    }
    else {
        // Not touching - reset timer
        robot.ballTouchTime = 0;
    }

    robot.wasTouchingBall = touching;

    // Check for boundary violations
    if (isOutOfBounds(robot.pos)) {
        robot.timeoutFrames = TIMEOUT_DURATION_FRAMES;
        cerr << "  ⚠ Robot out of bounds - 60s timeout initiated" << endl;
    }

    // Check for penalty area violations (complete entry)
    if (isInPenaltyArea(robot.pos, true)) {
        robot.timeoutFrames = TIMEOUT_DURATION_FRAMES;
        cerr << "  ⚠ Robot in penalty area - 60s timeout initiated" << endl;
    }

    // Warn about approaching holding limit
    if (robot.ballTouchTime > BALL_HOLD_TIME_LIMIT - 2.0 && touching) {
        cerr << "  ⚠ Ball hold time: " << robot.ballTouchTime << "s (limit: 10s)" << endl;
    }
}

/**
 * Update ball tracking each frame
 * Monitors stationarity for rule enforcement
 */
void updateBallTracking(Ball& ball) {
    if (isBallStationary(ball.vel)) {
        ball.stationaryTime += FRAME_TIME;
    }
    else {
        ball.stationaryTime = 0;
    }

    // Warn if ball stationary too long
    if (ball.stationaryTime > BALL_STATIONARY_LIMIT - 1.0) {
        cerr << "  ⚠ Ball stationary for " << ball.stationaryTime << "s (limit: 5s)" << endl;
    }

    ball.lastPos = ball.pos;
}

/**
 * Log a debug message with frame info
 */
void logDebug(const string& message) {
    cerr << "  " << message << endl;
}

/**
 * Log detailed robot state for debugging
 */
void logRobotState(const string& name, const Robot& robot) {
    cerr << "[" << name << "] pos: (" << fixed << setprecision(3)
        << robot.pos.x << ", " << robot.pos.z << ") "
        << "rot: " << (robot.rotY * 180.0 / M_PI) << "° "
        << "vel: " << robot.vel.length() << " m/s" << endl;
}