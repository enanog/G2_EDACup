/**
 * @file geometry.h
 * @brief Geometric calculations and utilities
 * @author Agustin Valenzuela,
 *         Alex Petersen,
 *         Dylan Frigerio,
 *         Enzo Fernandez Rosas
 *
 * @copyright Copyright (c) 2025
 */

#ifndef GEOMETRY_H
#define GEOMETRY_H

 // Calculate distance between two points
float distance(float x1, float z1, float x2, float z2);

// Calculate angle from one point to another
float angleTo(float fromX, float fromZ, float toX, float toZ);

// Normalize angle to [-PI, PI] range
float normalizeAngle(float angle);

// Calculate difference between two angles
float angleDifference(float angle1, float angle2);

#endif // GEOMETRY_H