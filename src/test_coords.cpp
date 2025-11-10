/**
 * @file simple_test.cpp
 * @brief Simplified testing program for EDACup robot motion validation
 *
 * This program tests basic robot movements and actions one at a time
 * to validate that communication with the simulator works correctly.
 *
 * Compile: g++ -std=c++17 simple_test.cpp -o simple_test
 * Usage: Update edacup.json with this binary and run the simulator
 *
 * @copyright Copyright (c) 2025
 */

#include <iostream>
#include <string>
#include <cmath>
#include <nlohmann/json.hpp>

using json = nlohmann::json;
using namespace std;

// ============================================================================
// CONSTANTS
// ============================================================================

// constexpr double M_PI = 3.14159265358979323846;
constexpr double FIELD_HALF_LENGTH = 1.095;
constexpr double FIELD_HALF_WIDTH = 0.79;
constexpr double RIGHT_GOAL_X = FIELD_HALF_LENGTH;
constexpr double LEFT_GOAL_X = -FIELD_HALF_LENGTH;

// ============================================================================
// SIMPLE DATA STRUCTURES
// ============================================================================

struct Vec2 {
	double x, z;
	Vec2(double x_ = 0, double z_ = 0) : x(x_), z(z_) {}
};

struct Robot {
	Vec2 pos;
	double rotY;
	Vec2 vel;
};

struct Ball {
	Vec2 pos;
	Vec2 vel;
};

struct GameState {
	Robot homeBot1;
	Robot homeBot2;
	Robot rivalBot1;
	Robot rivalBot2;
	Ball ball;
};

// ============================================================================
// TEST MODES
// ============================================================================

enum TestMode {
	TEST_FORWARD_MOTION,      // Move bot1 forward in +Z direction (rotY = 0)
	TEST_BACKWARD_MOTION,     // Move bot1 backward in -Z direction (rotY = π)
	TEST_STRAFE_LEFT,         // Move bot1 left in -X direction (rotY = -π/2)
	TEST_STRAFE_RIGHT,        // Move bot1 right in +X direction (rotY = π/2)
	TEST_ROTATION_CW,         // Rotate bot1 clockwise (negative rotY)
	TEST_ROTATION_CCW,        // Rotate bot1 counter-clockwise (positive rotY)
	TEST_CHASE_BALL,          // Bot1 chases ball (faces and moves toward it)
	TEST_DRIBBLER,            // Test dribbler while approaching ball
	TEST_KICK,                // Test kick when near ball
	TEST_CHIP,                // Test chip when near ball
	TEST_BOTH_ROBOTS,         // Move both robots simultaneously
	TEST_STAY_STILL,          // Don't move (verify position holding)
	TEST_AXIS_CROSS           // Move in cross pattern along both axes
};

// Current test mode (change this to test different behaviors)
TestMode g_currentTest = TEST_CHASE_BALL;

// Test state
bool g_isPlaying = false;
int g_frameCount = 0;

// ============================================================================
// COORDINATE SYSTEM CONSTANTS
// ============================================================================

/**
 * EDACup Simulator Coordinate System:
 *
 * - X axis: Field length (left goal at -X, right goal at +X)
 * - Z axis: Field depth (back at -Z, forward at +Z)
 * - rotY = 0 rad: Robot faces +Z (FORWARD)
 * - rotY = π/2 rad: Robot faces +X (RIGHT)
 * - rotY = -π/2 rad: Robot faces -X (LEFT)
 * - rotY = π rad: Robot faces -Z (BACKWARD)
 *
 * Positive rotation = counter-clockwise (left turn)
 * Negative rotation = clockwise (right turn)
 */

 // ============================================================================
 // UTILITY FUNCTIONS
 // ============================================================================

double normalizeAngle(double angle);
 /**
  * Calculate Euclidean distance between two points
  */
double distance(Vec2 a, Vec2 b) {
	double dx = b.x - a.x;
	double dz = b.z - a.z;
	return sqrt(dx * dx + dz * dz);
}

/**
 * Calculate the facing direction vector from rotY angle
 *
 * In the simulator:
 * - rotY = 0 → faces +Z (forward)
 * - rotY = π/2 → faces +X (right)
 *
 * Forward vector:
 * - forward_x = sin(rotY)
 * - forward_z = cos(rotY)
 */
Vec2 getFacingVector(double rotY) {
	return Vec2(cos(rotY), sin(rotY));
}

/**
 * Calculate required rotY angle to face from point 'from' to point 'to'
 *
 * Since rotY = 0 points toward +Z:
 * - Use atan2(dx, dz) instead of atan2(dz, dx)
 * - This gives angle measured from +Z axis (not +X axis)
 */
double angleToPoint(Vec2 from, Vec2 to) {
	double dx = to.x - from.x;
	double dz = to.z - from.z;
	// atan2(x, z) gives angle from +Z axis rotating toward +X
	return normalizeAngle(atan2(dx, dz) + M_PI);
}

/**
 * Normalize angle to [-π, π] range
 */
double normalizeAngle(double angle) {
	while (angle > M_PI) angle -= 2.0 * M_PI;
	while (angle < -M_PI) angle += 2.0 * M_PI;
	return angle;
}

// ============================================================================
// JSON PARSING
// ============================================================================

void parseRobot(const json& data, Robot& robot) {
	if (data.contains("position") && data["position"].is_array() && data["position"].size() >= 3) {
		robot.pos.x = data["position"][0];
		robot.pos.z = data["position"][2];
	}

	if (data.contains("rotation") && data["rotation"].is_array() && data["rotation"].size() >= 3) {
		robot.rotY = data["rotation"][1];
		// Don't add M_PI - use rotation as provided by simulator
	}

	if (data.contains("velocity") && data["velocity"].is_array() && data["velocity"].size() >= 3) {
		robot.vel.x = data["velocity"][0];
		robot.vel.z = data["velocity"][2];
	}
}

void parseBall(const json& data, Ball& ball) {
	if (data.contains("position") && data["position"].is_array() && data["position"].size() >= 3) {
		ball.pos.x = data["position"][0];
		ball.pos.z = data["position"][2];
	}

	if (data.contains("velocity") && data["velocity"].is_array() && data["velocity"].size() >= 3) {
		ball.vel.x = data["velocity"][0];
		ball.vel.z = data["velocity"][2];
	}
}

bool parseState(const json& message, GameState& state) {
	if (!message.contains("data")) return false;

	const json& data = message["data"];

	if (data.contains("homeBot1")) parseRobot(data["homeBot1"], state.homeBot1);
	if (data.contains("homeBot2")) parseRobot(data["homeBot2"], state.homeBot2);
	if (data.contains("rivalBot1")) parseRobot(data["rivalBot1"], state.rivalBot1);
	if (data.contains("rivalBot2")) parseRobot(data["rivalBot2"], state.rivalBot2);
	if (data.contains("ball")) parseBall(data["ball"], state.ball);

	return true;
}

// ============================================================================
// TEST BEHAVIORS
// ============================================================================

void testForwardMotion(const GameState& state, json& bot1, json& bot2) {
	// Move bot1 forward (toward +Z direction)
	// rotY = 0 means facing +Z (forward)
	double targetX = state.homeBot1.pos.x;
	double targetZ = state.homeBot1.pos.z + 0.5;  // Move forward in +Z

	bot1["positionXZ"] = { targetX, targetZ };
	bot1["rotationY"] = M_PI;  // Face +Z (forward)
	bot1["dribbler"] = 0.0;
	bot1["kick"] = 0.0;
	bot1["chip"] = 0.0;

	// Bot2 stays still
	bot2["positionXZ"] = { state.homeBot2.pos.x, state.homeBot2.pos.z };
	bot2["rotationY"] = state.homeBot2.rotY;
	bot2["dribbler"] = 0.0;
	bot2["kick"] = 0.0;
	bot2["chip"] = 0.0;

	if (g_frameCount % 50 == 0) {
		cerr << "TEST: Forward motion - Bot1 moving toward +Z (forward)" << endl;
		cerr << "  Position: (" << state.homeBot1.pos.x << ", " << state.homeBot1.pos.z << ")" << endl;
		cerr << "  Target: (" << targetX << ", " << targetZ << ")" << endl;
		cerr << "  Facing: +Z (rotY = 0)" << endl;
	}
}

void testBackwardMotion(const GameState& state, json& bot1, json& bot2) {
	// Move bot1 backward (toward -Z direction)
	// rotY = π means facing -Z (backward)
	double targetX = state.homeBot1.pos.x;
	double targetZ = state.homeBot1.pos.z - 0.5;  // Move backward in -Z

	bot1["positionXZ"] = { targetX, targetZ };
	bot1["rotationY"] = 0.0;  // Face -Z (backward)
	bot1["dribbler"] = 0.0;
	bot1["kick"] = 0.0;
	bot1["chip"] = 0.0;

	bot2["positionXZ"] = { state.homeBot2.pos.x, state.homeBot2.pos.z };
	bot2["rotationY"] = state.homeBot2.rotY;
	bot2["dribbler"] = 0.0;
	bot2["kick"] = 0.0;
	bot2["chip"] = 0.0;

	if (g_frameCount % 50 == 0) {
		cerr << "TEST: Backward motion - Bot1 moving toward -Z (backward)" << endl;
		cerr << "  Facing: -Z (rotY = π)" << endl;
	}
}

void testStrafeLeft(const GameState& state, json& bot1, json& bot2) {
	// Move bot1 left (toward -X direction)
	// rotY = -π/2 means facing -X (left)
	double targetX = state.homeBot1.pos.x - 0.5;  // Move left in -X
	double targetZ = state.homeBot1.pos.z;

	bot1["positionXZ"] = { targetX, targetZ };
	bot1["rotationY"] = M_PI / 2.0;  // Face -X (left)
	bot1["dribbler"] = 0.0;
	bot1["kick"] = 0.0;
	bot1["chip"] = 0.0;

	bot2["positionXZ"] = { state.homeBot2.pos.x, state.homeBot2.pos.z };
	bot2["rotationY"] = state.homeBot2.rotY;
	bot2["dribbler"] = 0.0;
	bot2["kick"] = 0.0;
	bot2["chip"] = 0.0;

	if (g_frameCount % 50 == 0) {
		cerr << "TEST: Strafe left - Bot1 moving toward -X (left)" << endl;
		cerr << "  Facing: -X (rotY = -π/2)" << endl;
	}
}

void testStrafeRight(const GameState& state, json& bot1, json& bot2) {
	// Move bot1 right (toward +X direction)
	// rotY = π/2 means facing +X (right)
	double targetX = state.homeBot1.pos.x + 0.5;  // Move right in +X
	double targetZ = state.homeBot1.pos.z;

	bot1["positionXZ"] = { targetX, targetZ };
	bot1["rotationY"] = -M_PI / 2.0;  // Face +X (right)
	bot1["dribbler"] = 0.0;
	bot1["kick"] = 0.0;
	bot1["chip"] = 0.0;

	bot2["positionXZ"] = { state.homeBot2.pos.x, state.homeBot2.pos.z };
	bot2["rotationY"] = state.homeBot2.rotY;
	bot2["dribbler"] = 0.0;
	bot2["kick"] = 0.0;
	bot2["chip"] = 0.0;

	if (g_frameCount % 50 == 0) {
		cerr << "TEST: Strafe right - Bot1 moving toward +X (right)" << endl;
		cerr << "  Facing: +X (rotY = π/2)" << endl;
	}
}

void testRotationCW(const GameState& state, json& bot1, json& bot2) {
	// Rotate clockwise (negative direction, turning right)
	// Start from current rotation and decrease by 0.02 rad per frame
	double targetRot = state.homeBot1.rotY - 0.5;  // CW rotation (more negative)
	targetRot = normalizeAngle(targetRot);

	bot1["positionXZ"] = { state.homeBot1.pos.x, state.homeBot1.pos.z };
	bot1["rotationY"] = targetRot;
	bot1["dribbler"] = 0.0;
	bot1["kick"] = 0.0;
	bot1["chip"] = 0.0;

	bot2["positionXZ"] = { state.homeBot2.pos.x, state.homeBot2.pos.z };
	bot2["rotationY"] = state.homeBot2.rotY;
	bot2["dribbler"] = 0.0;
	bot2["kick"] = 0.0;
	bot2["chip"] = 0.0;

	if (g_frameCount % 50 == 0) {
		cerr << "TEST: Clockwise rotation - Bot1 rotating right" << endl;
		cerr << "  Current rotation: " << state.homeBot1.rotY << " rad ("
			<< (state.homeBot1.rotY * 180.0 / M_PI) << "°)" << endl;
		cerr << "  Target rotation: " << targetRot << " rad" << endl;
		Vec2 facing = getFacingVector(state.homeBot1.rotY);
		cerr << "  Facing direction: (" << facing.x << ", " << facing.z << ")" << endl;
	}
}

void testRotationCCW(const GameState& state, json& bot1, json& bot2) {
	// Rotate counter-clockwise (positive direction, turning left)
	// Start from current rotation and increase by 0.02 rad per frame
	double targetRot = state.homeBot1.rotY + 0.5;  // CCW rotation (more positive)
	targetRot = normalizeAngle(targetRot);

	bot1["positionXZ"] = { state.homeBot1.pos.x, state.homeBot1.pos.z };
	bot1["rotationY"] = targetRot;
	bot1["dribbler"] = 0.0;
	bot1["kick"] = 0.0;
	bot1["chip"] = 0.0;

	bot2["positionXZ"] = { state.homeBot2.pos.x, state.homeBot2.pos.z };
	bot2["rotationY"] = state.homeBot2.rotY;
	bot2["dribbler"] = 0.0;
	bot2["kick"] = 0.0;
	bot2["chip"] = 0.0;

	if (g_frameCount % 50 == 0) {
		cerr << "TEST: Counter-clockwise rotation - Bot1 rotating left" << endl;
		cerr << "  Current rotation: " << state.homeBot1.rotY << " rad ("
			<< (state.homeBot1.rotY * 180.0 / M_PI) << "°)" << endl;
		cerr << "  Target rotation: " << targetRot << " rad" << endl;
		Vec2 facing = getFacingVector(state.homeBot1.rotY);
		cerr << "  Facing direction: (" << facing.x << ", " << facing.z << ")" << endl;
	}
}

void testChaseBall(const GameState& state, json& bot1, json& bot2) {
	// Move bot1 directly toward ball with correct facing
	double targetX = state.ball.pos.x;
	double targetZ = state.ball.pos.z;

	// Calculate angle to ball (corrected for rotY = 0 pointing to +Z)
	double angleToBall = angleToPoint(state.homeBot1.pos, state.ball.pos);

	bot1["positionXZ"] = { targetX, targetZ };
	bot1["rotationY"] = angleToBall;
	bot1["dribbler"] = 0.8;  // Activate dribbler
	bot1["kick"] = 0.0;
	bot1["chip"] = 0.0;

	bot2["positionXZ"] = { state.homeBot2.pos.x, state.homeBot2.pos.z };
	bot2["rotationY"] = state.homeBot2.rotY;
	bot2["dribbler"] = 0.0;
	bot2["kick"] = 0.0;
	bot2["chip"] = 0.0;

	if (g_frameCount % 50 == 0) {
		double dist = distance(state.homeBot1.pos, state.ball.pos);
		Vec2 facing = getFacingVector(angleToBall);
		cerr << "TEST: Chase ball" << endl;
		cerr << "  Ball position: (" << state.ball.pos.x << ", " << state.ball.pos.z << ")" << endl;
		cerr << "  Bot1 position: (" << state.homeBot1.pos.x << ", " << state.homeBot1.pos.z << ")" << endl;
		cerr << "  Distance to ball: " << dist << " m" << endl;
		cerr << "  Angle to ball: " << angleToBall << " rad ("
			<< (angleToBall * 180.0 / M_PI) << "°)" << endl;
		cerr << "  Should face: (" << facing.x << ", " << facing.z << ")" << endl;
	}
}

void testDribbler(const GameState& state, json& bot1, json& bot2) {
	// Chase ball with full dribbler power
	double targetX = state.ball.pos.x;
	double targetZ = state.ball.pos.z;
	double angleToBall = angleToPoint(state.homeBot1.pos, state.ball.pos);

	bot1["positionXZ"] = { targetX, targetZ };
	bot1["rotationY"] = angleToBall;
	bot1["dribbler"] = 1.0;  // Full dribbler power
	bot1["kick"] = 0.0;
	bot1["chip"] = 0.0;

	bot2["positionXZ"] = { state.homeBot2.pos.x, state.homeBot2.pos.z };
	bot2["rotationY"] = state.homeBot2.rotY;
	bot2["dribbler"] = 0.0;
	bot2["kick"] = 0.0;
	bot2["chip"] = 0.0;

	if (g_frameCount % 50 == 0) {
		double dist = distance(state.homeBot1.pos, state.ball.pos);
		cerr << "TEST: Dribbler - Full power dribbler active" << endl;
		cerr << "  Distance to ball: " << dist << " m" << endl;
	}
}

void testKick(const GameState& state, json& bot1, json& bot2) {
	double dist = distance(state.homeBot1.pos, state.ball.pos);

	if (dist < 0.15) {
		// Close enough to kick - face toward right goal
		double angleToGoal = angleToPoint(state.ball.pos, Vec2(RIGHT_GOAL_X, 0));

		bot1["positionXZ"] = { state.ball.pos.x, state.ball.pos.z };
		bot1["rotationY"] = angleToGoal;
		bot1["dribbler"] = 0.0;
		bot1["kick"] = 0.8;  // Kick power
		bot1["chip"] = 0.0;

		if (g_frameCount % 50 == 0) {
			Vec2 facing = getFacingVector(angleToGoal);
			cerr << "TEST: Kick - KICKING toward goal!" << endl;
			cerr << "  Facing angle: " << angleToGoal << " rad" << endl;
			cerr << "  Facing vector: (" << facing.x << ", " << facing.z << ")" << endl;
		}
	}
	else {
		// Move toward ball with correct facing
		double angleToBall = angleToPoint(state.homeBot1.pos, state.ball.pos);

		bot1["positionXZ"] = { state.ball.pos.x, state.ball.pos.z };
		bot1["rotationY"] = angleToBall;
		bot1["dribbler"] = 0.8;
		bot1["kick"] = 0.0;
		bot1["chip"] = 0.0;

		if (g_frameCount % 50 == 0) {
			cerr << "TEST: Kick - Approaching ball, distance: " << dist << " m" << endl;
		}
	}

	bot2["positionXZ"] = { state.homeBot2.pos.x, state.homeBot2.pos.z };
	bot2["rotationY"] = state.homeBot2.rotY;
	bot2["dribbler"] = 0.0;
	bot2["kick"] = 0.0;
	bot2["chip"] = 0.0;
}

void testChip(const GameState& state, json& bot1, json& bot2) {
	double dist = distance(state.homeBot1.pos, state.ball.pos);

	if (dist < 0.15) {
		// Close enough to chip - face toward right goal
		double angleToGoal = angleToPoint(state.ball.pos, Vec2(RIGHT_GOAL_X, 0));

		bot1["positionXZ"] = { state.ball.pos.x, state.ball.pos.z };
		bot1["rotationY"] = angleToGoal;
		bot1["dribbler"] = 0.0;
		bot1["kick"] = 0.0;
		bot1["chip"] = 0.6;  // Chip power

		if (g_frameCount % 50 == 0) {
			Vec2 facing = getFacingVector(angleToGoal);
			cerr << "TEST: Chip - CHIPPING toward goal!" << endl;
			cerr << "  Facing angle: " << angleToGoal << " rad" << endl;
			cerr << "  Facing vector: (" << facing.x << ", " << facing.z << ")" << endl;
		}
	}
	else {
		// Move toward ball with correct facing
		double angleToBall = angleToPoint(state.homeBot1.pos, state.ball.pos);

		bot1["positionXZ"] = { state.ball.pos.x, state.ball.pos.z };
		bot1["rotationY"] = angleToBall;
		bot1["dribbler"] = 0.8;
		bot1["kick"] = 0.0;
		bot1["chip"] = 0.0;

		if (g_frameCount % 50 == 0) {
			cerr << "TEST: Chip - Approaching ball, distance: " << dist << " m" << endl;
		}
	}

	bot2["positionXZ"] = { state.homeBot2.pos.x, state.homeBot2.pos.z };
	bot2["rotationY"] = state.homeBot2.rotY;
	bot2["dribbler"] = 0.0;
	bot2["kick"] = 0.0;
	bot2["chip"] = 0.0;
}

void testBothRobots(const GameState& state, json& bot1, json& bot2) {
	// Bot1 moves forward (+Z direction)
	bot1["positionXZ"] = { state.homeBot1.pos.x, state.homeBot1.pos.z + 0.3 };
	bot1["rotationY"] = 0.0;  // Face +Z (forward)
	bot1["dribbler"] = 0.0;
	bot1["kick"] = 0.0;
	bot1["chip"] = 0.0;

	// Bot2 moves backward (-Z direction)
	bot2["positionXZ"] = { state.homeBot2.pos.x, state.homeBot2.pos.z - 0.3 };
	bot2["rotationY"] = M_PI;  // Face -Z (backward)
	bot2["dribbler"] = 0.0;
	bot2["kick"] = 0.0;
	bot2["chip"] = 0.0;

	if (g_frameCount % 50 == 0) {
		cerr << "TEST: Both robots - Bot1 forward (+Z), Bot2 backward (-Z)" << endl;
	}
}

void testStayStill(const GameState& state, json& bot1, json& bot2) {
	// Both robots stay at current position
	bot1["positionXZ"] = { state.homeBot1.pos.x, state.homeBot1.pos.z };
	bot1["rotationY"] = state.homeBot1.rotY;
	bot1["dribbler"] = 0.0;
	bot1["kick"] = 0.0;
	bot1["chip"] = 0.0;

	bot2["positionXZ"] = { state.homeBot2.pos.x, state.homeBot2.pos.z };
	bot2["rotationY"] = state.homeBot2.rotY;
	bot2["dribbler"] = 0.0;
	bot2["kick"] = 0.0;
	bot2["chip"] = 0.0;

	if (g_frameCount % 100 == 0) {
		cerr << "TEST: Stay still - Robots holding position" << endl;
	}
}

/**
 * TEST_AXIS_CROSS - Move in cross pattern along both axes
 *
 * Movement sequence:
 * 1. Move +X (right) by 0.5m, face +X
 * 2. Return to origin, face -X
 * 3. Move -X (left) by 0.5m, face -X
 * 4. Return to origin, face +X
 * 5. Move +Z (forward) by 0.5m, face +Z
 * 6. Return to origin, face -Z
 * 7. Move -Z (backward) by 0.5m, face -Z
 * 8. Return to origin, face +Z
 * 9. Repeat cycle
 */
void testAxisCross(const GameState& state, json& bot1, json& bot2) {
	// State machine for cross pattern movement
	// Store initial position at frame 0
	static Vec2 origin(0, 0);
	static bool originSet = false;

	if (!originSet) {
		origin = state.homeBot1.pos;
		originSet = true;
		cerr << "TEST: Axis Cross - Starting at origin (" << origin.x << ", " << origin.z << ")" << endl;
	}

	// Each segment lasts 100 frames (5 seconds at 50ms/frame)
	const int FRAMES_PER_SEGMENT = 100;
	int segment = (g_frameCount / FRAMES_PER_SEGMENT) % 8;

	double targetX = origin.x;
	double targetZ = origin.z;
	double targetRot = 0.0;
	const char* description = "";

	switch (segment) {
	case 0:
		// Move +X (right)
		targetX = origin.x + 0.5;
		targetZ = origin.z;
		targetRot = M_PI / 2.0;  // Face +X (right)
		description = "Moving +X (right)";
		break;

	case 1:
		// Return from +X to origin
		targetX = origin.x;
		targetZ = origin.z;
		targetRot = -M_PI / 2.0;  // Face -X (toward origin)
		description = "Returning from +X to origin";
		break;

	case 2:
		// Move -X (left)
		targetX = origin.x - 0.5;
		targetZ = origin.z;
		targetRot = -M_PI / 2.0;  // Face -X (left)
		description = "Moving -X (left)";
		break;

	case 3:
		// Return from -X to origin
		targetX = origin.x;
		targetZ = origin.z;
		targetRot = M_PI / 2.0;  // Face +X (toward origin)
		description = "Returning from -X to origin";
		break;

	case 4:
		// Move +Z (forward)
		targetX = origin.x;
		targetZ = origin.z + 0.5;
		targetRot = 0.0;  // Face +Z (forward)
		description = "Moving +Z (forward)";
		break;

	case 5:
		// Return from +Z to origin
		targetX = origin.x;
		targetZ = origin.z;
		targetRot = M_PI;  // Face -Z (toward origin)
		description = "Returning from +Z to origin";
		break;

	case 6:
		// Move -Z (backward)
		targetX = origin.x;
		targetZ = origin.z - 0.5;
		targetRot = M_PI;  // Face -Z (backward)
		description = "Moving -Z (backward)";
		break;

	case 7:
		// Return from -Z to origin
		targetX = origin.x;
		targetZ = origin.z;
		targetRot = 0.0;  // Face +Z (toward origin)
		description = "Returning from -Z to origin";
		break;
	}

	bot1["positionXZ"] = { targetX, targetZ };
	bot1["rotationY"] = targetRot;
	bot1["dribbler"] = 0.0;
	bot1["kick"] = 0.0;
	bot1["chip"] = 0.0;

	bot2["positionXZ"] = { state.homeBot2.pos.x, state.homeBot2.pos.z };
	bot2["rotationY"] = state.homeBot2.rotY;
	bot2["dribbler"] = 0.0;
	bot2["kick"] = 0.0;
	bot2["chip"] = 0.0;

	// Log at segment transitions
	if (g_frameCount % FRAMES_PER_SEGMENT == 0) {
		Vec2 facing = getFacingVector(targetRot);
		cerr << "TEST: Axis Cross - Segment " << segment << ": " << description << endl;
		cerr << "  Current pos: (" << state.homeBot1.pos.x << ", " << state.homeBot1.pos.z << ")" << endl;
		cerr << "  Target pos: (" << targetX << ", " << targetZ << ")" << endl;
		cerr << "  Facing: " << targetRot << " rad (" << (targetRot * 180.0 / M_PI) << "°)" << endl;
		cerr << "  Facing vector: (" << facing.x << ", " << facing.z << ")" << endl;
	}
}

// ============================================================================
// MAIN TEST DISPATCHER
// ============================================================================

void runTest(const GameState& state, json& bot1, json& bot2) {
	switch (g_currentTest) {
	case TEST_FORWARD_MOTION:
		testForwardMotion(state, bot1, bot2);
		break;
	case TEST_BACKWARD_MOTION:
		testBackwardMotion(state, bot1, bot2);
		break;
	case TEST_STRAFE_LEFT:
		testStrafeLeft(state, bot1, bot2);
		break;
	case TEST_STRAFE_RIGHT:
		testStrafeRight(state, bot1, bot2);
		break;
	case TEST_ROTATION_CW:
		testRotationCW(state, bot1, bot2);
		break;
	case TEST_ROTATION_CCW:
		testRotationCCW(state, bot1, bot2);
		break;
	case TEST_CHASE_BALL:
		testChaseBall(state, bot1, bot2);
		break;
	case TEST_DRIBBLER:
		testDribbler(state, bot1, bot2);
		break;
	case TEST_KICK:
		testKick(state, bot1, bot2);
		break;
	case TEST_CHIP:
		testChip(state, bot1, bot2);
		break;
	case TEST_BOTH_ROBOTS:
		testBothRobots(state, bot1, bot2);
		break;
	case TEST_STAY_STILL:
		testStayStill(state, bot1, bot2);
		break;
	case TEST_AXIS_CROSS:
		testAxisCross(state, bot1, bot2);
		break;
	}
}

// ============================================================================
// MAIN LOOP
// ============================================================================

int main() {
	cerr << "========================================" << endl;
	cerr << "  EDACup Simple Test Program v2.0" << endl;
	cerr << "========================================" << endl;
	cerr << "Coordinate system:" << endl;
	cerr << "  rotY = 0     → +Z (forward)" << endl;
	cerr << "  rotY = π/2   → +X (right)" << endl;
	cerr << "  rotY = -π/2  → -X (left)" << endl;
	cerr << "  rotY = π     → -Z (backward)" << endl;
	cerr << "========================================" << endl;
	cerr << "Current test mode: ";

	switch (g_currentTest) {
	case TEST_FORWARD_MOTION: cerr << "FORWARD MOTION (+Z)"; break;
	case TEST_BACKWARD_MOTION: cerr << "BACKWARD MOTION (-Z)"; break;
	case TEST_STRAFE_LEFT: cerr << "STRAFE LEFT (-X)"; break;
	case TEST_STRAFE_RIGHT: cerr << "STRAFE RIGHT (+X)"; break;
	case TEST_ROTATION_CW: cerr << "ROTATION CLOCKWISE"; break;
	case TEST_ROTATION_CCW: cerr << "ROTATION COUNTER-CLOCKWISE"; break;
	case TEST_CHASE_BALL: cerr << "CHASE BALL"; break;
	case TEST_DRIBBLER: cerr << "DRIBBLER TEST"; break;
	case TEST_KICK: cerr << "KICK TEST"; break;
	case TEST_CHIP: cerr << "CHIP TEST"; break;
	case TEST_BOTH_ROBOTS: cerr << "BOTH ROBOTS"; break;
	case TEST_STAY_STILL: cerr << "STAY STILL"; break;
	case TEST_AXIS_CROSS: cerr << "AXIS CROSS PATTERN"; break;
	}
	cerr << endl;
	cerr << "========================================" << endl;
	cerr << endl;

	GameState state;

	while (true) {
		try {
			string line;
			if (!getline(cin, line)) break;
			if (line.empty()) continue;

			json message = json::parse(line);
			string type = message.value("type", "");

			if (type == "start") {
				g_isPlaying = true;
				g_frameCount = 0;
				cerr << ">>> GAME STARTED <<<" << endl;
			}
			else if (type == "stop") {
				g_isPlaying = false;
				cerr << ">>> GAME STOPPED <<<" << endl;
			}
			else if (type == "state" && g_isPlaying) {
				g_frameCount++;

				if (parseState(message, state)) {
					json bot1, bot2;
					runTest(state, bot1, bot2);

					json response = {
						{"type", "set"},
						{"data", {
							{"homeBot1", bot1},
							{"homeBot2", bot2}
						}}
					};

					cout << response.dump() << endl;
					cout.flush();
				}
			}

		}
		catch (const exception& e) {
			cerr << "ERROR: " << e.what() << endl;
		}
	}

	return 0;
}
