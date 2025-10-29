/**
 * @file geometry.cpp
 * @brief Implementation of geometric calculations and utility functions
 * @author Agustin Valenzuela,
 *         Alex Petersen,
 *         Dylan Frigerio,
 *         Enzo Fernandez Rosas
 *
 * @copyright Copyright (c) 2025
 */

#include "geometry.h"
#include "constants.h"
#include <cmath>

 // ============================================================================
 // DISTANCE CALCULATIONS
 // ============================================================================

float distance(float x1, float z1, float x2, float z2)
{
	float dx = x2 - x1;
	float dz = z2 - z1;
	return std::sqrt(dx * dx + dz * dz);
}

float distanceSquared(float x1, float z1, float x2, float z2)
{
	float dx = x2 - x1;
	float dz = z2 - z1;
	return dx * dx + dz * dz;
}

// ============================================================================
// ANGLE CALCULATIONS
// ============================================================================

float angleTo(float fromX, float fromZ, float toX, float toZ)
{
	float dx = toX - fromX;
	float dz = toZ - fromZ;

	// atan2(dx, dz) instead of atan2(dz, dx)
	float angle = std::atan2(dz, dx);

	// Keep it in range [-PI, PI]
	return normalizeAngle(angle);
}

float normalizeAngle(float angle)
{
	while (angle > 2 * M_PI) angle -= 2.0f * M_PI;
	while (angle < 0) angle += 2.0f * M_PI;
	return angle;
}

float angleDifference(float angle1, float angle2)
{
	return normalizeAngle(angle2 - angle1);
}

// ============================================================================
// POSITION VERIFICATION
// ============================================================================

bool isInsidePenaltyArea(float x, float z)
{
	// Check left penalty area (negative X)
	bool inLeftPenalty = (x >= LEFT_GOAL_X &&
		x <= LEFT_GOAL_X + PENALTY_AREA_LENGTH &&
		std::abs(z) <= PENALTY_AREA_WIDTH / 2.0f);

	// Check right penalty area (positive X)
	bool inRightPenalty = (x <= RIGHT_GOAL_X &&
		x >= RIGHT_GOAL_X - PENALTY_AREA_LENGTH &&
		std::abs(z) <= PENALTY_AREA_WIDTH / 2.0f);

	return inLeftPenalty || inRightPenalty;
}

bool isOutOfBounds(float x, float z)
{
	return (std::abs(x) > FIELD_HALF_LENGTH ||
		std::abs(z) > FIELD_HALF_WIDTH);
}

bool isInOwnHalf(float x)
{
	return x < 0.0f;
}

// ============================================================================
// VECTORS AND DIRECTIONS
// ============================================================================

void normalize(float& x, float& z)
{
	float magnitude = std::sqrt(x * x + z * z);

	if (magnitude > 0.0001f) {  // Avoid division by zero
		x /= magnitude;
		z /= magnitude;
	}
	else {
		x = 0.0f;
		z = 0.0f;
	}
}

float dotProduct(float x1, float z1, float x2, float z2)
{
	return x1 * x2 + z1 * z2;
}