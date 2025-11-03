/**
 * @file actions.cpp
 * @brief Implementation of robot actions
 * @author Agustin Valenzuela,
 *         Alex Petersen,
 *         Dylan Frigerio,
 *         Enzo Fernandez Rosas
 *
 * @copyright Copyright (c) 2025
 */

#include "actions.h"
#include "geometry.h"
#include "constants.h"
#include <cmath>
#include <algorithm>
#include <iostream>

 // Check if robot controls the ball
bool hasBallControl(const RobotState& robot, const RobotState& ball) {
    float dist = distance(robot.posX, robot.posZ, ball.posX, ball.posZ);
    return dist < BALL_CONTROL_DISTANCE;
}

// Check if robot can kick the ball
bool canKickBall(const RobotState& robot, const RobotState& ball) {
    float dist = distance(robot.posX, robot.posZ, ball.posX, ball.posZ);
    if (dist > BALL_CONTROL_DISTANCE) return false;

    float angleToBall = angleTo(robot.posX, robot.posZ, ball.posX, ball.posZ);
    float angleDiff = std::abs(angleDifference(robot.rotY, angleToBall));
    
	std::cerr << "Angle Diff = " << angleDiff << std::endl;

    return (angleDiff < 30.0f * DEG_TO_RAD);
}

// ============================================================================
// BALL CONTROL ACTIONS
// ============================================================================

RobotCommand BallControl::interceptBall(const RobotState& robot, const RobotState& ball) {
    RobotCommand cmd;
    cmd.targetX = ball.posX;
    cmd.targetZ = ball.posZ;
    cmd.targetRotY = angleTo(robot.posX, robot.posZ, ball.posX, ball.posZ);
    cmd.dribbler = 0.8f;
    cmd.kick = 0.0f;
    cmd.chip = 0.0f;
    return cmd;
}

RobotCommand BallControl::chaseBall(const RobotState& robot, const RobotState& ball) {
    RobotCommand cmd;
    cmd.targetX = ball.posX;
    cmd.targetZ = ball.posZ;
    cmd.targetRotY = angleTo(robot.posX, robot.posZ, ball.posX, ball.posZ);
    cmd.dribbler = 0.9f;
    cmd.kick = 0.0f;
    cmd.chip = 0.0f;
    return cmd;
}

RobotCommand BallControl::dribbleTo(const RobotState& robot, const RobotState& ball,
    float targetX, float targetZ) {
    RobotCommand cmd;

    // Move toward target in small steps
    float dirX = targetX - robot.posX;
    float dirZ = targetZ - robot.posZ;
    float len = std::sqrt(dirX * dirX + dirZ * dirZ);

    if (len > 0.01f) {
        dirX /= len;
        dirZ /= len;
    }

    const float STEP = 0.03f;
    cmd.targetX = robot.posX + dirX * STEP;
    cmd.targetZ = robot.posZ + dirZ * STEP;
    cmd.targetRotY = angleTo(robot.posX, robot.posZ, targetX, targetZ);
    cmd.dribbler = 1.0f;
    cmd.kick = 0.0f;
    cmd.chip = 0.0f;

    return cmd;
}

// ============================================================================
// SHOOTING ACTIONS
// ============================================================================

ActionResult Shooting::shootAtGoal(const RobotState& robot, const RobotState& ball) {
    ActionResult result;
    result.success = false;

    // Target center of opponent goal
    float targetX = RIGHT_GOAL_X;
    float targetZ = 0.0f;

    // Calculate kick power based on distance
    float distToGoal = distance(ball.posX, ball.posZ, targetX, targetZ);
    float power = std::min(distToGoal / 2.0f, 1.0f);

    if (!canKickBall(robot, ball)) {
        // Position behind ball for shooting
        float behindX = ball.posX - 0.12f;
        float behindZ = ball.posZ;
        float angle = angleTo(ball.posX, ball.posZ, targetX, targetZ);

        result.command.targetX = behindX;
        result.command.targetZ = behindZ;
        result.command.targetRotY = angle;
        result.command.dribbler = 0.8f;
        result.command.kick = 0.0f;
		result.command.chip = 0.0f;
        result.description = "Positioning for shot";
        return result;
    }

    // Execute shot with kick
    result.success = true;
    result.command.targetX = ball.posX;
    result.command.targetZ = ball.posZ;
    result.command.targetRotY = angleTo(ball.posX, ball.posZ, targetX, targetZ);
    result.command.dribbler = 0.3f;
    result.command.kick = std::clamp(power * 1.3f, 0.6f, 1.0f);
    result.command.chip = 0.0f;
    result.description = "Shooting at goal";

    std::cerr << ">>> EXECUTING SHOT! Power: " << result.command.kick << " <<<" << std::endl;
    return result;
}

RobotCommand Shooting::clearBall(const RobotState& robot, const RobotState& ball) {
    RobotCommand cmd;

    // Kick ball toward opponent goal
    float targetX = RIGHT_GOAL_X;
    float targetZ = (ball.posZ > 0) ? -FIELD_HALF_WIDTH : FIELD_HALF_WIDTH;

    if (canKickBall(robot, ball)) {
        cmd.targetX = ball.posX;
        cmd.targetZ = ball.posZ;
        cmd.targetRotY = angleTo(ball.posX, ball.posZ, targetX, targetZ);
        cmd.dribbler = 0.0f;
        cmd.kick = 1.0f;
        cmd.chip = 0.0f;
    }
    else {
        // Position behind ball for clearing
        float behindX = ball.posX - 0.12f;
        float behindZ = ball.posZ;
        cmd.targetX = behindX;
        cmd.targetZ = behindZ;
        cmd.targetRotY = angleTo(ball.posX, ball.posZ, targetX, targetZ);
        cmd.dribbler = 0.5f;
        cmd.kick = 0.0f;
        cmd.chip = 0.0f;
    }

    return cmd;
}

// ============================================================================
// POSITIONING ACTIONS
// ============================================================================

RobotCommand Positioning::moveTo(const RobotState& robot, float targetX, float targetZ) {
    RobotCommand cmd;
    cmd.targetX = targetX;
    cmd.targetZ = targetZ;
    cmd.targetRotY = angleTo(robot.posX, robot.posZ, targetX, targetZ);
    cmd.dribbler = 1.0f;
    cmd.kick = 0.0f;
    cmd.chip = 0.0f;
    return cmd;
}