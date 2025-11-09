/**
 * @file geometry.cpp
 * @brief Implementation of geometric calculations
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

 /**
  * Calculate Euclidean distance between two points
  */
float distance(float x1, float z1, float x2, float z2)
{
    float dx = x2 - x1;
    float dz = z2 - z1;
    return std::sqrt(dx * dx + dz * dz);
}

/**
 * Calculate angle from one point to another
 *
 * IMPORTANT: In EDACup coordinate system:
 * - rotY = 0 means robot faces +X direction (toward opponent goal)
 * - We use atan2(dz, dx) to get proper angle
 *
 * The function adds M_PI because the simulator expects angles
 * offset by 180 degrees from standard mathematical convention
 */
float angleTo(float fromX, float fromZ, float toX, float toZ)
{
    float dx = toX - fromX;
    float dz = toZ - fromZ;

    // Calculate angle in standard coordinates
    // atan2(dz, dx) gives angle from +X axis
    float angle = std::atan2(dx, dz) + M_PI;

    // Normalize to [-PI, PI] range
    return normalizeAngle(angle);
}

/**
 * Normalize angle to [-PI, PI] range
 */
float normalizeAngle(float angle)
{
    while (angle > M_PI) angle -= 2.0f * M_PI;
    while (angle < -M_PI) angle += 2.0f * M_PI;
    return angle;
}

/**
 * Get facing direction vector from rotY angle
 * rotY = 0 → faces +X (opponent goal direction)
 * rotY = PI/2 → faces +Z
 */
Vec2 getFacingVector(float rotY) {
    return Vec2(cos(rotY), sin(rotY));
}

/**
 * Calculate shortest angular difference between two angles
 * Result is in [-PI, PI] range
 */
float angleDifference(float angle1, float angle2)
{
    return normalizeAngle(angle2 - angle1);
}

/**
 * Smooth rotation toward target angle
 * maxDelta: maximum rotation per frame (radians)
 */
float smoothRotation(float currentAngle, float targetAngle, float maxDelta) {
    float diff = angleDifference(currentAngle, targetAngle);

    // Clamp difference to maxDelta
    if (std::abs(diff) <= maxDelta) {
        return targetAngle;
    }

    // Move toward target by maxDelta
    if (diff > 0) {
        return normalizeAngle(currentAngle + maxDelta);
    }
    else {
        return normalizeAngle(currentAngle - maxDelta);
    }
}