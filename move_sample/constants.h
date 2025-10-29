/**
 * @file constants.h
 * @brief Constants and field specifications for EDACup 2025 robot competition
 * @author Agustin Valenzuela,
 *         Alex Petersen,
 *         Dylan Frigerio,
 *         Enzo Fernandez Rosas
 *
 * @copyright Copyright (c) 2025
 */

#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <cmath>

 // ============================================================================
 // CONVERSIONS
 // ============================================================================

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define DEG_TO_RAD (3.1415926535F / 180.0F)
#define RAD_TO_DEG (180.0f / M_PI)

// ============================================================================
// FIELD DIMENSIONS (CORREGIDAS SEGÚN DIAGRAMA)
// ============================================================================
const float FIELD_LENGTH = 2.19f;				// 219 cm in meters
const float FIELD_WIDTH = 1.58f;				// 158 cm in meters
const float FIELD_HALF_LENGTH = 1.095f;
const float FIELD_HALF_WIDTH = 0.79f;

// ============================================================================
// GOAL
// ============================================================================
const float GOAL_WIDTH = 0.60f;					// 60 cm
const float GOAL_HEIGHT = 0.10f;				// 10 cm
const float GOAL_DEPTH = 0.074f;				// 74 mm
const float GOAL_HALF_WIDTH = GOAL_WIDTH / 2.0f;

// Goal positions (center of goal)
const float LEFT_GOAL_X = -FIELD_HALF_LENGTH;   // Nuestro arco (izquierda)
const float RIGHT_GOAL_X = FIELD_HALF_LENGTH;   // Arco rival (derecha)
const float GOAL_Z = 0.0f;	// Centered on Z axis

// ============================================================================
// PENALTY AREAS
// ============================================================================
const float PENALTY_AREA_WIDTH = 0.25f;			// 25 cm
const float PENALTY_AREA_LENGTH = 0.80f;		// 80 cm
const float PENALTY_AREA_CORNER_RADIUS = 0.15f; // 15 cm
const float PENALTY_AREA_LINE_WIDTH = 0.020f;   // 20 mm

// ============================================================================
// CENTER CIRCLE
// ============================================================================
const float CENTER_CIRCLE_DIAMETER = 0.60f;		// 60 cm
const float CENTER_CIRCLE_RADIUS = CENTER_CIRCLE_DIAMETER / 2.0f;

// ============================================================================
// NEUTRAL POINTS
// ============================================================================
// Field center
const float NEUTRAL_CENTER_X = 0.0f;
const float NEUTRAL_CENTER_Z = 0.0f;

// Points near corners (45 cm from long edge)
const float NEUTRAL_CORNER_DISTANCE = 0.45f;	// 45 cm

// ============================================================================
// WEDGE (ramp at borders)
// ============================================================================
const float WEDGE_BASE = 0.10f;					// 10 cm
const float WEDGE_HEIGHT = 0.02f;				// 2 cm

// ============================================================================
// BALL
// ============================================================================
const float BALL_WEIGHT = 0.046f;				// 46 g in kg
const float BALL_DIAMETER = 0.043f;				// 43 mm in meters
const float BALL_RADIUS = BALL_DIAMETER / 2.0f;

// ============================================================================
// ROBOT
// ============================================================================
const float ROBOT_HEIGHT = 0.140f;				// 140 mm
const float ROBOT_DIAMETER = 0.180f;			// 180 mm
const float ROBOT_RADIUS = ROBOT_DIAMETER / 2.0f;
const float ROBOT_WEIGHT = 2.6f;				// 2.6 kg
const float ROBOT_CENTER_OF_MASS = 0.035f;		// 35 mm from floor

// ============================================================================
// ROBOT CONTROLS
// ============================================================================
const float DRIBBLER_MIN = 0.0f;
const float DRIBBLER_MAX = 1.0f;
const float KICK_MIN = 0.0f;
const float KICK_MAX = 1.0f;
const float CHIP_MIN = 0.0f;
const float CHIP_MAX = 1.0f;
const float CHIP_ANGLE = 45.0f * DEG_TO_RAD;	// 45 degrees

// ============================================================================
// TIMING AND FREQUENCIES
// ============================================================================
const float STATE_UPDATE_INTERVAL = 0.050f;		// 50 ms
const int STATE_UPDATE_FREQUENCY = 20;			// 20 Hz

// ============================================================================
// GAME RULES
// ============================================================================
const int MATCH_DURATION_MINUTES = 5;
const int MATCH_HALVES = 2;
const int AUTO_WIN_GOALS = 10;
const float MIN_DISTANCE_FROM_BALL_KICKOFF = 0.30f;	// 30 cm
const float PENALTY_TIME_SECONDS = 60.0f;			// 1 minute

// ============================================================================
// USEFUL DISTANCES
// ============================================================================
const float SAFE_DISTANCE_FROM_WALL = 0.15f;		// Safe distance from walls
const float BALL_CONTROL_DISTANCE = 0.12f;			// Distance to consider ball controlled

#endif // CONSTANTS_H