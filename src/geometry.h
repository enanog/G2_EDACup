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

 /**
  * 2D Vector structure
  */
struct Vec2 {
    float x, z;

    Vec2() : x(0), z(0) {}
    Vec2(float x_, float z_) : x(x_), z(z_) {}
};

/**
 * Calculate distance between two points
 */
float distance(float x1, float z1, float x2, float z2);

/**
 * Calculate angle from one point to another
 * Returns angle in radians [-PI, PI]
 */
float angleTo(float fromX, float fromZ, float toX, float toZ);

/**
 * Normalize angle to [-PI, PI] range
 */
float normalizeAngle(float angle);

/**
 * Calculate difference between two angles
 * Returns shortest angular distance in [-PI, PI]
 */
float angleDifference(float angle1, float angle2);

/**
 * Get facing direction vector from rotation angle
 */
Vec2 getFacingVector(float rotY);

/**
 * Smooth rotation toward target angle
 * maxDelta: maximum rotation change per frame
 */
float smoothRotation(float currentAngle, float targetAngle, float maxDelta);

#endif // GEOMETRY_H