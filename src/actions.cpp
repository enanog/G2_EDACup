// ============================================================================
// actions.cpp
// Implementation of low-level robot actions
// ============================================================================

#include "actions.h"
#include "geometry.h"
#include "utils.h"

/**
 * Move robot to target position with specified rotation
 * Applies safety checks and smooth control
 */
void moveTo(const Robot& robot, Vec2 target, double targetRotation, json& command) {
    // Ensure target is safe (within bounds, outside penalty areas)
    Vec2 safeTarget = getSafeTarget(target, robot.pos);
    
    // Smooth rotation toward target
    double smoothRot = smoothRotation(robot.rotY, targetRotation, SMOOTH_ROTATION_SPEED);
    
    // Set command
    command["positionXZ"] = { safeTarget.x, safeTarget.z };
    command["rotationY"] = smoothRot;
    command["dribbler"] = 0.0;
    command["kick"] = 0.0;
    command["chip"] = 0.0;
}

/**
 * Rotate robot to face target point without translating
 */
void faceTarget(const Robot& robot, Vec2 target, json& command) {
    double targetAngle = angleToPoint(robot.pos, target);
    double smoothRot = smoothRotation(robot.rotY, targetAngle, SMOOTH_ROTATION_SPEED);
    
    command["positionXZ"] = { robot.pos.x, robot.pos.z };
    command["rotationY"] = smoothRot;
    command["dribbler"] = 0.0;
    command["kick"] = 0.0;
    command["chip"] = 0.0;
}

/**
 * Stop robot at current position (no movement or actions)
 */
void stop(const Robot& robot, json& command) {
    command["positionXZ"] = { robot.pos.x, robot.pos.z };
    command["rotationY"] = robot.rotY;
    command["dribbler"] = 0.0;
    command["kick"] = 0.0;
    command["chip"] = 0.0;
}

/**
 * Move with dribbler active to control the ball
 */
void dribble(const Robot& robot, Vec2 target, double rotation, double power, json& command) {
    Vec2 safeTarget = getSafeTarget(target, robot.pos);
    double smoothRot = smoothRotation(robot.rotY, rotation, SMOOTH_ROTATION_SPEED);
    
    command["positionXZ"] = { safeTarget.x, safeTarget.z };
    command["rotationY"] = smoothRot;
    command["dribbler"] = clamp(power, 0.0, 1.0);
    command["kick"] = 0.0;
    command["chip"] = 0.0;
}

/**
 * Execute a kick toward target
 */
void kick(const Robot& robot, Vec2 target, double rotation, double power, json& command) {
    Vec2 safeTarget = getSafeTarget(target, robot.pos);
    
    command["positionXZ"] = { safeTarget.x, safeTarget.z };
    command["rotationY"] = rotation;
    command["dribbler"] = 0.0;
    command["kick"] = clamp(power, 0.0, 1.0);
    command["chip"] = 0.0;
}

/**
 * Execute a chip kick (lofted) toward target
 */
void chip(const Robot& robot, Vec2 target, double rotation, double power, json& command) {
    Vec2 safeTarget = getSafeTarget(target, robot.pos);
    
    command["positionXZ"] = { safeTarget.x, safeTarget.z };
    command["rotationY"] = rotation;
    command["dribbler"] = 0.0;
    command["kick"] = 0.0;
    command["chip"] = clamp(power, 0.0, 1.0);
}

/**
 * Set basic movement without any special actions
 */
void setBasicMovement(Vec2 position, double rotation, json& command) {
    command["positionXZ"] = { position.x, position.z };
    command["rotationY"] = rotation;
    command["dribbler"] = 0.0;
    command["kick"] = 0.0;
    command["chip"] = 0.0;
}

/**
 * Check if robot has been holding ball too long and should release
 */
bool shouldBackOff(const Robot& robot) {
    return robot.ballTouchTime > (BALL_HOLD_TIME_LIMIT - 1.5);
}
