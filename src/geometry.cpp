// ============================================================================
// geometry.cpp
// Implementation of geometric and mathematical utilities
// ============================================================================

#include "geometry.h"
#include <algorithm>

// Vec2 constructor
Vec2::Vec2(double x_, double z_) : x(x_), z(z_) {}

// Vector addition
Vec2 Vec2::operator+(const Vec2& other) const {
    return Vec2(x + other.x, z + other.z);
}

// Vector subtraction
Vec2 Vec2::operator-(const Vec2& other) const {
    return Vec2(x - other.x, z - other.z);
}

// Scalar multiplication
Vec2 Vec2::operator*(double scalar) const {
    return Vec2(x * scalar, z * scalar);
}

// Calculate vector magnitude (length)
double Vec2::length() const {
    return std::sqrt(x * x + z * z);
}

// Return normalized (unit) vector
Vec2 Vec2::normalized() const {
    double len = length();
    // Avoid division by zero
    if (len < 0.001) return Vec2(1, 0);
    return Vec2(x / len, z / len);
}

// Dot product of two vectors
double Vec2::dot(const Vec2& other) const {
    return x * other.x + z * other.z;
}

// Calculate distance between two points
double distance(Vec2 a, Vec2 b) {
    return (b - a).length();
}

// Normalize angle to [-π, π] range for consistent angle representation
double normalizeAngle(double angle) {
    // Handle angles outside the standard range
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

/**
 * Get facing direction vector from rotation angle
 * In EDACup: rotY = 0 means facing +X axis
 *            rotY = π/2 means facing +Z axis
 */
Vec2 getFacingVector(double rotY) {
    return Vec2(std::cos(rotY), std::sin(rotY));
}

/**
 * Calculate angle needed to face from one point to another
 * Uses atan2 to get angle from +X axis (since rotY=0 is +X direction)
 */
double angleToPoint(Vec2 from, Vec2 to) {
    double dx = to.x - from.x;
    double dz = to.z - from.z;
    // atan2(z, x) gives angle from +X axis in XZ plane
    return std::atan2(dz, dx);
}

/**
 * Smoothly rotate from current angle toward target angle
 * Prevents instant snapping by limiting rotation per frame
 */
double smoothRotation(double currentAngle, double targetAngle, double maxChange) {
    // Calculate shortest angular distance
    double diff = normalizeAngle(targetAngle - currentAngle);
    // Clamp the change to maxChange
    diff = std::max(-maxChange, std::min(maxChange, diff));
    // Apply the limited rotation
    return normalizeAngle(currentAngle + diff);
}

// Linear interpolation between two values
double lerp(double a, double b, double t) {
    return a + (b - a) * t;
}

// Clamp value to specified range
double clamp(double value, double minVal, double maxVal) {
    return std::max(minVal, std::min(maxVal, value));
}
