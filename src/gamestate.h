/**
 * @file gamestate.h
 * @brief Global game state manager with strategy execution
 * @author Agustin Valenzuela, Alex Petersen, Dylan Frigerio, Enzo Fernandez Rosas
 * @copyright Copyright (c) 2025
 */

#ifndef GAMESTATE_H
#define GAMESTATE_H

#include <cstdint>
#include <nlohmann/json.hpp>
#include <vector>

#include "ball.h"
#include "constants.h"
#include "robot.h"
#include "field.h"

using json = nlohmann::json;

/**
 * @brief Player indices for robot vector
 */
enum Player {
    NONE = -1,
    HOMEBOT_1 = 0,
    HOMEBOT_2 = 1,
    RIVALBOT_1 = 2,
    RIVALBOT_2 = 3,
    PLAYER_COUNT = 4
};

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

    RoleAssignment() : bot1Role(Role::STRIKER), bot2Role(Role::DEFENDER) {
    }
    RoleAssignment(Role b1, Role b2) : bot1Role(b1), bot2Role(b2) {
    }
};

/**
 * @class GameState
 * @brief Global game state manager
 *
 * Thread Safety Note:
 * - This class is NOT internally thread-safe
 * - All access must be protected by external mutex (see main.cpp)
 * - parseFromJson() and update() should never be called concurrently
 *
 * Manages all robots, ball, roles, and executes strategy decisions
 */
class GameState {
  private:
    // Unified robot storage
    std::vector<Robot> playerList_;
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

    /**
     * @brief Get robot by player enum
     */
    Robot& getBot(Player player) {
        return playerList_[player];
    }

    const Robot& getBot(Player player) const {
        return playerList_[player];
    }

    // Convenience accessors
    Robot& getHomeBot1() {
        return playerList_[HOMEBOT_1];
    }
    Robot& getHomeBot2() {
        return playerList_[HOMEBOT_2];
    }
    Robot& getRivalBot1() {
        return playerList_[RIVALBOT_1];
    }
    Robot& getRivalBot2() {
        return playerList_[RIVALBOT_2];
    }

    const Robot& getHomeBot1() const {
        return playerList_[HOMEBOT_1];
    }
    const Robot& getHomeBot2() const {
        return playerList_[HOMEBOT_2];
    }
    const Robot& getRivalBot1() const {
        return playerList_[RIVALBOT_1];
    }
    const Robot& getRivalBot2() const {
        return playerList_[RIVALBOT_2];
    }

    Ball& getBall() {
        return ball_;
    }
    const Ball& getBall() const {
        return ball_;
    }

    uint32_t getFrameCount() const {
        return frameCount_;
    }
    const RoleAssignment& getRoles() const {
        return currentRoles_;
    }
    const FieldMap& getFieldMap() const {
        return fieldMap_;
    }

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
    void nextFrame() {
        frameCount_++;
    }

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

    // ========================================================================
    // INTERACTIONS
    // ========================================================================

    /**
     * @brief Check which player has control of the ball
     * @return Player enum of controlling robot, or -1 if none
     */
    Player whoHasBallControl() const;

    /**
     * @brief Execute quick pass between robots
     */
    int8_t quickPass();
    /**
     * @brief Execute quick chip between robots
     */
    int8_t quickChip();
};

#endif  // GAMESTATE_H
