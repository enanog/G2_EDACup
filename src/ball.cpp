/**
 * @file Ball.cpp
 * @brief Implementation of Ball class
 * @author Agustin Valenzuela, Alex Petersen, Dylan Frigerio, Enzo Fernandez Rosas
 * @copyright Copyright (c) 2025
 */

#include "ball.h"

#include <algorithm>

#include "robot.h"

// ============================================================================
// CONSTRUCTORS
// ============================================================================

Ball::Ball()
	: posX_(0),
	  posY_(0),
	  posZ_(0),
	  rotX_(0),
	  rotY_(0),
	  rotZ_(0),
	  velX_(0),
	  velY_(0),
	  velZ_(0),
	  angVelX_(0),
	  angVelY_(0),
	  angVelZ_(0) {
}

Ball::Ball(float x, float y, float z)
	: posX_(x),
	  posY_(y),
	  posZ_(z),
	  rotX_(0),
	  rotY_(0),
	  rotZ_(0),
	  velX_(0),
	  velY_(0),
	  velZ_(0),
	  angVelX_(0),
	  angVelY_(0),
	  angVelZ_(0) {
}

// ============================================================================
// UTILITY METHODS
// ============================================================================

float Ball::getSpeed() const {
	return std::sqrt(velX_ * velX_ + velZ_ * velZ_);
}

void Ball::predictPosition(float deltaTime, float& outX, float& outZ) const {
	outX = posX_ + velX_ * deltaTime;
	outZ = posZ_ + velZ_ * deltaTime;

	// Clamp to field boundaries
	outX = std::clamp(outX, -FIELD_HALF_LENGTH + 0.105f, FIELD_HALF_LENGTH - 0.105f);
	outZ = std::clamp(outZ, -FIELD_HALF_WIDTH + 0.105f, FIELD_HALF_WIDTH - 0.105f);
}

float Ball::distanceTo(float x, float z) const {
	float dx = x - posX_;
	float dz = z - posZ_;
	return std::sqrt(dx * dx + dz * dz);
}

float Ball::distanceTo(const Robot& robot) const {
	return distanceTo(robot.getPosX(), robot.getPosZ());
}
