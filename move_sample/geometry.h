/**
 * @file geometry.h
 * @brief Geometric calculations and utility functions for robot positioning and movement
 * @author Agustin Valenzuela,
 *         Alex Petersen,
 *         Dylan Frigerio,
 *         Enzo Fernandez Rosas
 *
 * @copyright Copyright (c) 2025
 */

#ifndef GEOMETRY_H
#define GEOMETRY_H

 // ============================================================================
 // DISTANCE CALCULATIONS
 // ============================================================================

 /**
  * @brief Calculate Euclidean distance between two points
  * @param x1 X coordinate of first point
  * @param z1 Z coordinate of first point
  * @param x2 X coordinate of second point
  * @param z2 Z coordinate of second point
  * @return Distance between the two points
  */
float distance(float x1, float z1, float x2, float z2);

/**
 * @brief Calculate squared distance between two points (faster for comparisons)
 * @param x1 X coordinate of first point
 * @param z1 Z coordinate of first point
 * @param x2 X coordinate of second point
 * @param z2 Z coordinate of second point
 * @return Squared distance between the two points
 */
float distanceSquared(float x1, float z1, float x2, float z2);

// ============================================================================
// ANGLE CALCULATIONS
// ============================================================================

/**
 * @brief Calculate angle from one point to another
 * @param fromX X coordinate of starting point
 * @param fromZ Z coordinate of starting point
 * @param toX X coordinate of target point
 * @param toZ Z coordinate of target point
 * @return Angle in radians
 */
float angleTo(float fromX, float fromZ, float toX, float toZ);

/**
 * @brief Normalize angle to range [-PI, PI]
 * @param angle Angle in radians
 * @return Normalized angle
 */
float normalizeAngle(float angle);

/**
 * @brief Calculate shortest difference between two angles
 * @param angle1 First angle in radians
 * @param angle2 Second angle in radians
 * @return Shortest angular difference
 */
float angleDifference(float angle1, float angle2);

// ============================================================================
// POSITION VERIFICATION
// ============================================================================

/**
 * @brief Check if a position is inside the penalty area
 * @param x X coordinate
 * @param z Z coordinate
 * @return true if inside penalty area, false otherwise
 */
bool isInsidePenaltyArea(float x, float z);

/**
 * @brief Check if a position is out of bounds
 * @param x X coordinate
 * @param z Z coordinate
 * @return true if out of bounds, false otherwise
 */
bool isOutOfBounds(float x, float z);

/**
 * @brief Check if a position is in own half of the field
 * @param x X coordinate
 * @param isLeftTeam true if checking for left team, false for right team
 * @return true if in own half, false otherwise
 */
bool isInOwnHalf(float x);

// ============================================================================
// VECTORS AND DIRECTIONS
// ============================================================================

/**
 * @brief Normalize a 2D vector to unit length
 * @param x Reference to X component (will be modified)
 * @param z Reference to Z component (will be modified)
 */
void normalize(float& x, float& z);

/**
 * @brief Calculate dot product of two 2D vectors
 * @param x1 X component of first vector
 * @param z1 Z component of first vector
 * @param x2 X component of second vector
 * @param z2 Z component of second vector
 * @return Dot product
 */
float dotProduct(float x1, float z1, float x2, float z2);

#endif // GEOMETRY_H