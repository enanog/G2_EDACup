// ============================================================================
// strategy.cpp
// Implementation of game strategy and decision-making
// ============================================================================

#include "strategy.h"
#include "actions.h"
#include "geometry.h"
#include "utils.h"
#include <iostream>

using namespace std;

// Strategy constants
constexpr double BALL_CONTROL_DISTANCE = 0.15;   // Distance to start dribbling
constexpr double KICK_DISTANCE = 0.12;           // Distance to shoot
constexpr double SHOT_ALIGNMENT_THRESHOLD = 0.25; // Radians (~14 degrees)
constexpr double DEFENDER_BASE_Z = -0.75;        // Defender default Z position (near own goal)
constexpr double DEFENDER_MAX_FORWARD = -0.30;   // Don't push too far forward
constexpr double DEFENDER_DANGER_DISTANCE = 0.40; // Ball distance to trigger aggressive defense

/**
 * ATTACKER STRATEGY
 *
 * Field orientation:
 * - Own goal: Z = -1.095 (backward)
 * - Opponent goal: Z = +1.095 (forward)
 *
 * States:
 * 1. TIMEOUT: Robot is in penalty, stay still
 * 2. BACKING_OFF: Too long holding ball, release it
 * 3. SHOOTING: Close to ball, aligned with goal, take shot
 * 4. DRIBBLING: Have ball control, moving toward goal
 * 5. APPROACHING: Moving toward ball to gain control
 */
void executeAttacker(const GameState& state, json& command) {
    const Robot& bot = state.homeBot1;
    const Ball& ball = state.ball;

    // STATE 1: Handle timeout (out of bounds or penalty violation)
    if (bot.timeoutFrames > 0) {
        stop(bot, command);
        if (state.frameCount % 100 == 0) {
            int secondsLeft = bot.timeoutFrames / 20;
            cerr << "[Attacker] IN TIMEOUT - " << secondsLeft << "s remaining" << endl;
        }
        return;
    }

    // STATE 2: Back off if holding ball too long
    if (shouldBackOff(bot)) {
        Vec2 backoffDir = getFacingVector(bot.rotY) * -1.0; // Reverse direction
        Vec2 backoffTarget = bot.pos + (backoffDir * 0.25);
        moveTo(bot, backoffTarget, bot.rotY, command);

        if (state.frameCount % 50 == 0) {
            cerr << "[Attacker] BACKING OFF - avoid holding violation" << endl;
        }
        return;
    }

    // Calculate tactical information
    double distToBall = distance(bot.pos, ball.pos);
    Vec2 goalPos(0.0, FORWARD_GOAL_Z); // Opponent goal center at +Z
    double angleToGoal = angleToPoint(ball.pos, goalPos);
    double angleToBall = angleToPoint(bot.pos, ball.pos);

    // STATE 3: SHOOTING - Close enough and well-aligned
    if (distToBall < KICK_DISTANCE) {
        double targetAngle = angleToGoal;
        bool aligned = isAlignedForShot(bot, ball.pos, goalPos, targetAngle);

        if (aligned && shouldShoot(bot, ball, targetAngle)) {
            // TAKE THE SHOT!
            kick(bot, ball.pos, targetAngle, 0.85, command);

            if (state.frameCount % 10 == 0) {
                cerr << "[Attacker] ⚽ SHOOTING! Distance: " << distToBall
                    << "m, Angle: " << (targetAngle * 180 / M_PI) << "°" << endl;
            }
        }
        else {
            // Adjust alignment before shooting
            dribble(bot, ball.pos, targetAngle, 0.7, command);

            if (state.frameCount % 50 == 0) {
                double angleDiff = abs(normalizeAngle(bot.rotY - targetAngle));
                cerr << "[Attacker] ALIGNING for shot - angle diff: "
                    << (angleDiff * 180 / M_PI) << "°" << endl;
            }
        }
        return;
    }

    // STATE 4: DRIBBLING - Have ball control, move toward goal
    if (distToBall < BALL_CONTROL_DISTANCE) {
        // Calculate dribble target (push ball toward goal)
        Vec2 toGoal = (goalPos - ball.pos).normalized();
        Vec2 dribbleTarget = ball.pos + (toGoal * 0.10);

        dribble(bot, dribbleTarget, angleToGoal, 0.8, command);

        if (state.frameCount % 50 == 0) {
            cerr << "[Attacker] DRIBBLING toward goal - dist: "
                << distance(ball.pos, goalPos) << "m" << endl;
        }
        return;
    }

    // STATE 5: APPROACHING - Move toward ball to gain control
    moveTo(bot, ball.pos, angleToBall, command);

    if (state.frameCount % 50 == 0) {
        cerr << "[Attacker] APPROACHING ball - distance: " << distToBall << "m" << endl;
    }
}

/**
 * DEFENDER STRATEGY
 *
 * Field orientation:
 * - Own goal: Z = -1.095 (backward, must protect)
 * - Opponent goal: Z = +1.095 (forward)
 *
 * States:
 * 1. TIMEOUT: Robot is in penalty
 * 2. CLEARING: Ball is very close, kick it away
 * 3. AGGRESSIVE: Ball is dangerously close to goal, intercept
 * 4. POSITIONING: Track ball laterally, maintain defensive position
 */
void executeDefender(const GameState& state, json& command) {
    const Robot& bot = state.homeBot2;
    const Ball& ball = state.ball;

    // STATE 1: Handle timeout
    if (bot.timeoutFrames > 0) {
        stop(bot, command);
        if (state.frameCount % 100 == 0) {
            int secondsLeft = bot.timeoutFrames / 20;
            cerr << "[Defender] IN TIMEOUT - " << secondsLeft << "s remaining" << endl;
        }
        return;
    }

    // Calculate defensive metrics
    Vec2 ownGoal(0.0, BACKWARD_GOAL_Z);  // Own goal at -Z
    double ballDistToGoal = distance(ball.pos, ownGoal);
    double botDistToBall = distance(bot.pos, ball.pos);

    // Get optimal defender position
    Vec2 defenderPos = getDefenderPosition(ball);
    double angleToFaceBall = angleToPoint(defenderPos, ball.pos);

    // STATE 2: CLEARING - Ball is very close, kick it away immediately
    if (botDistToBall < KICK_DISTANCE && ballDistToGoal < 0.5) {
        // Kick ball toward opponent side (+Z direction)
        Vec2 clearTarget(ball.pos.x, FORWARD_GOAL_Z);
        double clearAngle = angleToPoint(ball.pos, clearTarget);

        kick(bot, ball.pos, clearAngle, 0.75, command);

        if (state.frameCount % 20 == 0) {
            cerr << "[Defender] ⚡ CLEARING ball away from goal!" << endl;
        }
        return;
    }

    // STATE 3: AGGRESSIVE - Ball is dangerously close to goal
    if (ballDistToGoal < DEFENDER_DANGER_DISTANCE) {
        // Move aggressively toward ball to intercept
        Vec2 interceptTarget = ball.pos + (ball.vel * 0.1); // Predict ball movement
        double interceptAngle = angleToPoint(bot.pos, interceptTarget);

        dribble(bot, interceptTarget, interceptAngle, 0.6, command);

        if (state.frameCount % 50 == 0) {
            cerr << "[Defender] AGGRESSIVE intercept - ball danger zone: "
                << ballDistToGoal << "m from goal" << endl;
        }
        return;
    }

    // STATE 4: POSITIONING - Maintain defensive position, track ball
    moveTo(bot, defenderPos, angleToFaceBall, command);

    if (state.frameCount % 100 == 0) {
        cerr << "[Defender] POSITIONING - tracking ball at X: "
            << ball.pos.x << ", defender X: " << defenderPos.x << endl;
    }
}

/**
 * Check if robot is well-aligned for a shot on goal
 * Returns true if angle difference is within threshold
 */
bool isAlignedForShot(const Robot& robot, Vec2 ballPos, Vec2 goalPos, double& angleToGoal) {
    angleToGoal = angleToPoint(ballPos, goalPos);
    double currentFacing = robot.rotY;
    double angleDiff = abs(normalizeAngle(currentFacing - angleToGoal));

    return angleDiff < SHOT_ALIGNMENT_THRESHOLD;
}

/**
 * Determine if conditions are right to take a shot
 * Checks alignment and distance
 */
bool shouldShoot(const Robot& robot, const Ball& ball, double alignmentAngle) {
    double distToBall = distance(robot.pos, ball.pos);
    double angleDiff = abs(normalizeAngle(robot.rotY - alignmentAngle));

    // Must be close and well-aligned
    return distToBall < KICK_DISTANCE && angleDiff < SHOT_ALIGNMENT_THRESHOLD;
}

/**
 * Calculate optimal defender position based on ball location
 * Defender stays between ball and own goal (at -Z), tracking ball's X position
 */
Vec2 getDefenderPosition(const Ball& ball) {
    // Base Z position (stay near goal at -Z)
    double defZ = DEFENDER_BASE_Z;

    // If ball is on our side (negative Z) and moving toward goal, push forward slightly
    if (ball.pos.z < 0 && ball.vel.z < -0.1) {
        defZ = DEFENDER_MAX_FORWARD;
    }

    // Track ball's X position (lateral movement) but with some damping
    double defX = ball.pos.x * 0.7; // Don't fully track, maintain center bias

    // Clamp X to reasonable range (stay away from sidelines)
    defX = clamp(defX, -FIELD_HALF_WIDTH + 0.2, FIELD_HALF_WIDTH - 0.2);

    return Vec2(defX, defZ);
}