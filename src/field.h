/**
 * @file field.h
 * @brief Field mapping and tactical awareness system
 * @author Agustin Valenzuela, Alex Petersen, Dylan Frigerio, Enzo Fernandez Rosas
 * @copyright Copyright (c) 2025
 */

#ifndef FIELD_H
#define FIELD_H

#include "ball.h"
#include "constants.h"
#include "robot.h"

/**
 * @struct FieldMap
 * @brief Field mapping for tactical awareness
 *
 * Tracks penalty areas, predicted ball position, threats, and tactical information.
 * Updated every frame to provide current game state analysis for strategy decisions.
 */
struct FieldMap {
    // ========================================================================
    // PENALTY AREA BOUNDARIES
    // ========================================================================

    float ownPenaltyMinX;    ///< Minimum X coordinate of own penalty area
    float ownPenaltyMaxX;    ///< Maximum X coordinate of own penalty area
    float oppPenaltyMinX;    ///< Minimum X coordinate of opponent penalty area
    float oppPenaltyMaxX;    ///< Maximum X coordinate of opponent penalty area
    float penaltyMinZ;       ///< Minimum Z coordinate of penalty areas
    float penaltyMaxZ;       ///< Maximum Z coordinate of penalty areas

    // ========================================================================
    // PREDICTED POSITIONS (LATENCY COMPENSATION)
    // ========================================================================

    float predictedBallX;    ///< Predicted ball X position after delay
    float predictedBallZ;    ///< Predicted ball Z position after delay
    float predictedBallVelX; ///< Ball velocity in X direction
    float predictedBallVelZ; ///< Ball velocity in Z direction

    // ========================================================================
    // TACTICAL INFORMATION
    // ========================================================================

    float ballDistToOwnGoal; ///< Distance from ball to own goal
    float ballDistToOppGoal; ///< Distance from ball to opponent goal
    bool isBallStuck;        ///< True if ball has been stationary for >5 seconds
    int activeOpponents;     ///< Number of opponent robots currently on field

    // ========================================================================
    // THREAT ASSESSMENT
    // ========================================================================

    float closestRivalToOwnGoal; ///< Distance of closest rival to own goal
    const Robot* closestRivalPtr; ///< Pointer to closest rival robot

    /**
     * @brief Constructor - initializes all fields to default values
     */
    FieldMap();

    /**
     * @brief Update field map with current game state
     * @param ball Current ball state
     * @param rival1 First opponent robot
     * @param rival2 Second opponent robot
     *
     * Updates all tactical information including penalty areas, ball prediction,
     * distances, active opponents, and threat assessment. Should be called once
     * per frame before strategy execution.
     */
    void update(const Ball& ball, const Robot& rival1, const Robot& rival2);

    /**
     * @brief Check if position is inside own penalty area
     * @param x X coordinate to check
     * @param z Z coordinate to check
     * @return True if position is inside own penalty area
     */
    bool inOwnPenaltyArea(float x, float z) const;

    /**
     * @brief Check if position is inside opponent penalty area
     * @param x X coordinate to check
     * @param z Z coordinate to check
     * @return True if position is inside opponent penalty area
     */
    bool inOppPenaltyArea(float x, float z) const;

    /**
     * @brief Get safe position outside penalty area
     * @param x X coordinate (modified if inside penalty area)
     * @param z Z coordinate (modified if inside penalty area)
     *
     * If the provided position is inside a penalty area, adjusts it to the
     * nearest safe position outside the area with a 0.1m margin.
     */
    void getSafePosition(float& x, float& z) const;

    /**
     * @brief Check if pass line is clear (no obstacles)
     * @param passer Robot attempting the pass
     * @param receiver Robot receiving the pass
     * @param rival1 First opponent robot
     * @param rival2 Second opponent robot
     * @return True if there is a clear path between passer and receiver
     */
    bool isPassLineClear(const Robot& passer,
                         const Robot& receiver,
                         const Robot& rival1,
                         const Robot& rival2) const;

    /**
     * @brief Calculate pass quality score
     * @param passer Robot attempting the pass
     * @param receiver Robot receiving the pass
     * @param rival1 First opponent robot
     * @param rival2 Second opponent robot
     * @return Quality score: 0.0 = bad pass, 1.0 = excellent pass
     *
     * Considers distance (optimal ~1.0m), path clearance, and forward progress.
     * Returns 0.0 if receiver is off-field, too close (<0.25m), too far (>1.2m),
     * or path is blocked.
     */
    float calculatePassQuality(const Robot& passer,
                              const Robot& receiver,
                              const Robot& rival1,
                              const Robot& rival2) const;

    /**
     * @brief Check if shooting path to goal is blocked by rivals
     * @param shooter Robot attempting to shoot
     * @param rival1 First opponent robot
     * @param rival2 Second opponent robot
     * @return True if path to goal is blocked
     */
    bool isShootingPathBlocked(const Robot& shooter,
                              const Robot& rival1,
                              const Robot& rival2) const;

    /**
     * @brief Check if ball is stuck (stationary for >5 seconds)
     * @return True if ball has been stuck for more than 5 seconds
     */
    bool isBallStuck() const {
        return isballStuck;
    }
};

#endif  // FIELD_H
