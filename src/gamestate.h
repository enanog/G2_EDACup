/**
 * @file GameState.h
 * @brief Global game state manager with strategy execution
 * @author Agustin Valenzuela, Alex Petersen, Dylan Frigerio, Enzo Fernandez Rosas
 * @copyright Copyright (c) 2025
 */

#ifndef GAMESTATE_H
#define GAMESTATE_H

#include "Robot.h"
#include "Ball.h"
#include "constants.h"
#include <cstdint>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

/**
 * @brief Robot roles in dynamic strategy
 */
enum class Role {
    STRIKER,   // Attack, shoot, pass forward
    DEFENDER,  // Protect goal, mark threats, support
    OFFFIELD   // Robot not on field
};

/**
 * @brief Role assignment for both home robots
 */
struct RoleAssignment {
    Role bot1Role;
    Role bot2Role;

    RoleAssignment() : bot1Role(Role::STRIKER), bot2Role(Role::DEFENDER) {}
    RoleAssignment(Role b1, Role b2) : bot1Role(b1), bot2Role(b2) {}
};

/**
 * @brief Field mapping for tactical awareness
 * 
 * Tracks penalty areas, predicted ball position, threats, and tactical info
 */
struct FieldMap {
    // Penalty area boundaries
    float ownPenaltyMinX;
    float ownPenaltyMaxX;
    float oppPenaltyMinX;
    float oppPenaltyMaxX;
    float penaltyMinZ;
    float penaltyMaxZ;

    // Predicted positions (latency compensation)
    float predictedBallX;
    float predictedBallZ;
    float predictedBallVelX;
    float predictedBallVelZ;

    // Tactical information
    float ballDistToOwnGoal;
    float ballDistToOppGoal;
    int activeOpponents;

    // Threat assessment
    float closestRivalToOwnGoal;
    const Robot* closestRivalPtr;

    FieldMap();
    
    /**
     * @brief Update field map with current game state
     */
    void update(const Ball& ball, const Robot& rival1, const Robot& rival2);
    
    /**
     * @brief Check if position is inside own penalty area
     */
    bool inOwnPenaltyArea(float x, float z) const;
    
    /**
     * @brief Check if position is inside opponent penalty area
     */
    bool inOppPenaltyArea(float x, float z) const;
    
    /**
     * @brief Get safe position outside penalty area
     */
    void getSafePosition(float& x, float& z) const;
    
    /**
     * @brief Check if pass line is clear (no obstacles)
     */
    bool isPassLineClear(const Robot& passer, const Robot& receiver,
                         const Robot& rival1, const Robot& rival2) const;
    
    /**
     * @brief Calculate pass quality score (0.0 = bad, 1.0 = excellent)
     */
    float calculatePassQuality(const Robot& passer, const Robot& receiver,
                              const Robot& rival1, const Robot& rival2) const;
    
    /**
     * @brief Check if shooting path is blocked by rivals
     */
    bool isShootingPathBlocked(const Robot& shooter,
                              const Robot& rival1,
                              const Robot& rival2) const;
};

/**
 * @class GameState
 * @brief Global game state manager
 * 
 * Manages all robots, ball, roles, and executes strategy decisions
 */
class GameState {
private:
    // Robots and ball
    Robot homeBot1_;
    Robot homeBot2_;
    Robot rivalBot1_;
    Robot rivalBot2_;
    Ball ball_;
    
    // Strategy state
    RoleAssignment currentRoles_;
    FieldMap fieldMap_;
    
    // Frame counter
    uint32_t frameCount_;

    // Strategy constants
    static constexpr float PREDICTION_TIME = 0.15f;
    static constexpr float ROLE_HYSTERESIS = 0.15f;
    static constexpr float MIN_PASS_DISTANCE = 0.25f;
    static constexpr float MAX_PASS_DISTANCE = 1.2f;
    static constexpr float DANGER_ZONE_RADIUS = 0.7f;
    static constexpr float CHIP_OBSTACLE_THRESHOLD = 0.40f;

    /**
     * @brief Assign roles dynamically based on game state
     */
    void assignRoles();
    
    /**
     * @brief Execute striker behavior
     */
    void executeStrikerLogic(Robot& striker, const Robot& defender);
    
    /**
     * @brief Execute defender behavior
     */
    void executeDefenderLogic(Robot& defender, const Robot& striker);

public:
    GameState();

    // ========================================================================
    // STATE ACCESS
    // ========================================================================
    
    Robot& getHomeBot1() { return homeBot1_; }
    Robot& getHomeBot2() { return homeBot2_; }
    Robot& getRivalBot1() { return rivalBot1_; }
    Robot& getRivalBot2() { return rivalBot2_; }
    Ball& getBall() { return ball_; }
    
    const Robot& getHomeBot1() const { return homeBot1_; }
    const Robot& getHomeBot2() const { return homeBot2_; }
    const Robot& getRivalBot1() const { return rivalBot1_; }
    const Robot& getRivalBot2() const { return rivalBot2_; }
    const Ball& getBall() const { return ball_; }
    
    uint32_t getFrameCount() const { return frameCount_; }
    const RoleAssignment& getRoles() const { return currentRoles_; }
    const FieldMap& getFieldMap() const { return fieldMap_; }

    // ========================================================================
    // STATE MANAGEMENT
    // ========================================================================
    
    /**
     * @brief Parse game state from JSON message
     */
    void parseFromJson(const json& message);
    
    /**
     * @brief Reset state for new game
     */
    void reset();
    
    /**
     * @brief Increment frame counter
     */
    void nextFrame() { frameCount_++; }

    // ========================================================================
    // STRATEGY EXECUTION
    // ========================================================================
    
    /**
     * @brief Execute strategy and update robot commands
     * This is the main decision-making function called each frame
     */
    void update();
    
    /**
     * @brief Create JSON message with robot commands
     */
    json createCommandJson() const;
};

#endif // GAMESTATE_H