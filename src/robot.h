/**
 * @file Robot.h
 * @brief Robot class with movement, ball control, and shooting capabilities
 * @author Agustin Valenzuela, Alex Petersen, Dylan Frigerio, Enzo Fernandez Rosas
 * @copyright Copyright (c) 2025
 */

#ifndef ROBOT_H
#define ROBOT_H

#include "constants.h"
#include <cmath>
#include <algorithm>

 // Forward declaration
class Ball;

/**
 * @class Robot
 * @brief Represents a soccer robot with state and tactical actions
 *
 * Manages:
 * - Position, rotation, and velocity
 * - Movement and positioning commands
 * - Ball interaction (control, dribble, shoot, clear)
 * - Tactical queries (distance, line of sight)
 */
class Robot {
private:
    // ========================================================================
    // STATE VARIABLES
    // ========================================================================

    // Position
    float posX_, posY_, posZ_;

    // Rotation (radians)
    float rotX_, rotY_, rotZ_;

    // Linear velocity
    float velX_, velY_, velZ_;

    // Angular velocity
    float angVelX_, angVelY_, angVelZ_;

    // ========================================================================
    // CONTROL OUTPUTS
    // ========================================================================

    float targetX_, targetZ_;      // Target movement position
    float targetRotY_;             // Target rotation angle
    float dribbler_;               // Dribbler power [0-1]
    float kick_;                   // Kick power [0-1]
    float chip_;                   // Chip kick power [0-1]

    // ========================================================================
    // PRIVATE UTILITY METHODS
    // ========================================================================

    /**
     * @brief Calculate angle to target point
     */
    float angleTo(float toX, float toZ) const;

    /**
     * @brief Calculate angular difference between two angles
     */
    float angleDifference(float angle1, float angle2) const;

    /**
     * @brief Smoothly interpolate toward target angle
     */
    float smoothRotation(float currentAngle, float targetAngle, float speed) const;

    /**
     * @brief Clamp coordinates to valid field boundaries
     */
    void clampToFieldBounds(float& x, float& z) const;

public:
    // ========================================================================
    // CONSTRUCTORS
    // ========================================================================

    Robot();
    Robot(float x, float y, float z);

    // ========================================================================
    // STATE GETTERS
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
    // CONTROL GETTERS
    // ========================================================================

    float getTargetX() const { return targetX_; }
    float getTargetZ() const { return targetZ_; }
    float getTargetRotY() const { return targetRotY_; }
    float getDribbler() const { return dribbler_; }
    float getKick() const { return kick_; }
    float getChip() const { return chip_; }

    // ========================================================================
    // STATE SETTERS
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
    void setAngularVelocity(float x, float y, float z) {
        angVelX_ = x; angVelY_ = y; angVelZ_ = z;
    }

    // ========================================================================
    // STATE QUERIES
    // ========================================================================

    /**
     * @brief Check if robot is on the field (Y >= 0.5)
     */
    bool isOnField() const { return posY_ >= 0.5f; }

    /**
     * @brief Calculate distance to another robot
     */
    float distanceTo(const Robot& other) const;

    /**
     * @brief Calculate distance to a point
     */
    float distanceTo(float x, float z) const;

    /**
     * @brief Check if robot has control of the ball
     */
    bool hasBallControl(const Ball& ball) const;

    /**
     * @brief Check if robot can kick the ball (proper position and angle)
     */
    bool canKickBall(const Ball& ball) const;

    /**
     * @brief Check if there's a clear path to target (no opponents blocking)
     */
    bool hasClearPath(float targetX, float targetZ,
        const Robot& opp1, const Robot& opp2) const;

    // ========================================================================
    // BASIC CONTROL ACTIONS
    // ========================================================================

    /**
     * @brief Reset all control outputs to zero
     */
    void resetControls();

    /**
     * @brief Hold current position
     */
    void holdPosition();

    /**
     * @brief Move to target position
     */
    void moveTo(float targetX, float targetZ);

    /**
     * @brief Rotate to face a target point
     */
    void faceTowards(float pointX, float pointZ);

    /**
     * @brief Move to position while facing a specific point
     */
    void moveToWhileFacing(float targetX, float targetZ, float faceX, float faceZ);

    // ========================================================================
    // BALL INTERACTION ACTIONS
    // ========================================================================

    /**
     * @brief Intercept moving ball (predicts position)
     */
    void interceptBall(const Ball& ball);

    /**
     * @brief Chase ball to gain control
     */
    void chaseBall(const Ball& ball);

    /**
     * @brief Dribble ball toward target position
     */
    void dribbleTo(const Ball& ball, float targetX, float targetZ);

    /**
     * @brief Dribble ball toward opponent's goal
     */
    void dribbleToGoal(const Ball& ball);

    /**
     * @brief Shoot ball at goal
     * @return True if shot was executed, false if still positioning
     */
    bool shootAtGoal(const Ball& ball);

    /**
     * @brief Clear ball away from goal (defensive)
     */
    void clearBall(const Ball& ball);
};

#endif // ROBOT_H