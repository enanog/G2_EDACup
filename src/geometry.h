// ============================================================================
// geometry.h
// Mathematical utilities for 2D vector operations and angle calculations
// ============================================================================

#ifndef GEOMETRY_H
#define GEOMETRY_H

#include <cmath>

// Mathematical constant
constexpr double M_PI = 3.14159265358979323846;

/**
 * 2D Vector structure with common operations
 */
struct Vec2 {
    double x, z;
    
    // Constructors
    Vec2(double x_ = 0, double z_ = 0);
    
    // Vector arithmetic operators
    Vec2 operator+(const Vec2& other) const;
    Vec2 operator-(const Vec2& other) const;
    Vec2 operator*(double scalar) const;
    
    // Vector operations
    double length() const;           // Get magnitude of vector
    Vec2 normalized() const;         // Get unit vector in same direction
    double dot(const Vec2& other) const;  // Dot product
};

/**
 * Calculate Euclidean distance between two points
 */
double distance(Vec2 a, Vec2 b);

/**
 * Normalize angle to [-π, π] range
 */
double normalizeAngle(double angle);

/**
 * Calculate the facing direction vector from rotY angle
 * rotY = 0 → faces +X (right)
 * rotY = π/2 → faces +Z (forward)
 */
Vec2 getFacingVector(double rotY);

/**
 * Calculate required rotY angle to face from 'from' point to 'to' point
 */
double angleToPoint(Vec2 from, Vec2 to);

/**
 * Smoothly interpolate from current angle to target angle
 * maxChange limits the rotation speed
 */
double smoothRotation(double currentAngle, double targetAngle, double maxChange);

/**
 * Linear interpolation between two values
 */
double lerp(double a, double b, double t);

/**
 * Clamp value between min and max
 */
double clamp(double value, double minVal, double maxVal);

#endif // GEOMETRY_H
