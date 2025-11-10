/**
 * @file constants.h
 * @brief Game constants and field dimensions
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

 // Math constants
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
#define DEG_TO_RAD (M_PI / 180.0f)

// Field dimensions
const float FIELD_HALF_LENGTH = 1.095f;
const float FIELD_HALF_WIDTH = 0.79f;

// Goal positions
const float LEFT_GOAL_X = -FIELD_HALF_LENGTH;
const float RIGHT_GOAL_X = FIELD_HALF_LENGTH;
const float GOAL_HALF_WIDTH = 0.30f;

// Robot and ball parameters
const float ROBOT_DIAMETER = 0.180f;
const float BALL_CONTROL_DISTANCE = 0.12f;

const float PENALTY_AREA_DEPTH = 0.25f;
const float PENALTY_AREA_HALF_WIDTH = 0.40f;

#endif // CONSTANTS_H
