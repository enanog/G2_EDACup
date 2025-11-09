/**
 * @file Ball.h
 * @brief Ball class with physics and prediction capabilities
 * @author Agustin Valenzuela, Alex Petersen, Dylan Frigerio, Enzo Fernandez Rosas
 * @copyright Copyright (c) 2025
 */

#ifndef BALL_H
#define BALL_H

#include "constants.h"
#include <cmath>

/**
 * @class Ball
 * @brief Represents the soccer ball with position, velocity, and prediction
 */
class Ball {
private:
    // Position
    float posX_, posY_, posZ_;
    
    // Rotation (in radians)
    float rotX_, rotY_, rotZ_;
    
    // Linear velocity
    float velX_, velY_, velZ_;
    
    // Angular velocity
    float angVelX_, angVelY_, angVelZ_;

public:
    /**
     * @brief Default constructor - initializes ball at origin
     */
    Ball();
    
    /**
     * @brief Constructor with position
     * @param x Initial X position
     * @param y Initial Y position
     * @param z Initial Z position
     */
    Ball(float x, float y, float z);

    // ========================================================================
    // GETTERS
    // ========================================================================
    
    float getPosX() const { return posX_; }
    float getPosY() const { return posY_; }
    float getPosZ() const { return posZ_; }
    
    float getRotX() const { return rotX_; }
    float getRotY() const { return rotY_; }
    float getRotZ() const { return rotZ_; }
    
    float getVelX() const { return velX_; }
    float getVelY() const { return velY_; }
    float getVelZ() const { return velZ_; }
    
    float getAngVelX() const { return angVelX_; }
    float getAngVelY() const { return angVelY_; }
    float getAngVelZ() const { return angVelZ_; }

    // ========================================================================
    // SETTERS
    // ========================================================================
    
    void setPosX(float x) { posX_ = x; }
    void setPosY(float y) { posY_ = y; }
    void setPosZ(float z) { posZ_ = z; }
    void setPosition(float x, float y, float z) { posX_ = x; posY_ = y; posZ_ = z; }
    
    void setRotX(float x) { rotX_ = x; }
    void setRotY(float y) { rotY_ = y; }
    void setRotZ(float z) { rotZ_ = z; }
    void setRotation(float x, float y, float z) { rotX_ = x; rotY_ = y; rotZ_ = z; }
    
    void setVelX(float x) { velX_ = x; }
    void setVelY(float y) { velY_ = y; }
    void setVelZ(float z) { velZ_ = z; }
    void setVelocity(float x, float y, float z) { velX_ = x; velY_ = y; velZ_ = z; }
    
    void setAngVelX(float x) { angVelX_ = x; }
    void setAngVelY(float y) { angVelY_ = y; }
    void setAngVelZ(float z) { angVelZ_ = z; }
    void setAngularVelocity(float x, float y, float z) { angVelX_ = x; angVelY_ = y; angVelZ_ = z; }

    // ========================================================================
    // UTILITY METHODS
    // ========================================================================
    
    /**
     * @brief Get current speed (magnitude of velocity)
     * @return Speed in m/s
     */
    float getSpeed() const;
    
    /**
     * @brief Predict ball position after a given time
     * @param deltaTime Time in seconds to predict ahead
     * @param outX Predicted X position (output)
     * @param outZ Predicted Z position (output)
     */
    void predictPosition(float deltaTime, float& outX, float& outZ) const;
    
    /**
     * @brief Calculate distance to a point
     * @param x Target X coordinate
     * @param z Target Z coordinate
     * @return Distance in meters
     */
    float distanceTo(float x, float z) const;
    
    /**
     * @brief Calculate distance to a robot
     * @param robot Target robot
     * @return Distance in meters
     */
    float distanceTo(const class Robot& robot) const;
};

#endif // BALL_H