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

float distance(float x1, float z1, float x2, float z2)
{
    float dx = x2 - x1;
    float dz = z2 - z1;
    return std::sqrt(dx * dx + dz * dz);
}

float angleTo(float fromX, float fromZ, float toX, float toZ)
{
    float dx = toX - fromX;
    float dz = toZ - fromZ;
    return std::atan2(dx, dz);
}

float normalizeAngle(float angle)
{
    while (angle > M_PI) angle -= 2.0f * M_PI;
    while (angle < -M_PI) angle += 2.0f * M_PI;
    return angle;
}

float angleDifference(float angle1, float angle2)
{
    return normalizeAngle(angle2 - angle1);
}