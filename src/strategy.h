/**
 * @file strategy.h
 * @brief Strategy management and role assignment system
 * @author Agustin Valenzuela, Alex Petersen, Dylan Frigerio, Enzo Fernandez Rosas
 * @copyright Copyright (c) 2025
 */

#ifndef STRATEGY_H
#define STRATEGY_H

#include "ball.h"
#include "field.h"
#include "robot.h"

// Forward declaration
class GameState;

/**
 * @brief Robot roles in dynamic strategy
 */
enum class Roles {
	DEFENDER,     // Goalkeeper - protects own goal
	INTERCEPTOR,  // Follows the ball when not in possession
	CAMPER,       // Midfielder (when ball is stuck)
	SUPPORT,      // Assists and distributes ball
	ATTACKER,     // Applies pressure on rival's side
	STRIKER,      // Positioned to receive passes and score
	OFFIELD       // Off-field (disabled)
};

/**
 * @brief Available game strategies
 */
enum class Strategies {
	DEFENSIVE,          // One defender, one interceptor
	BALANCED,           // One defender, one attacker
    CAMPING,            // One camper, one interceptor
	OFFENSIVE,          // One support, one striker
	OFFIELD_STRATEGY,   // One or both robots off field
	NO_STRATEGY         // No robots on field
};

/**
 * @brief Role assignment for both home robots
 */
struct RoleAssignment {
	Roles bot1Role;
	Roles bot2Role;
	Strategies currentStrategy;

	RoleAssignment()
		: bot1Role(Roles::DEFENDER),
		  bot2Role(Roles::INTERCEPTOR),
		  currentStrategy(Strategies::DEFENSIVE) {}
};

/**
 * @class StrategyManager
 * @brief Manages strategy selection and role execution
 *
 * This class encapsulates all strategy-related logic, separating it from GameState.
 * It determines which strategy to use based on game conditions and executes
 * the appropriate behaviors for each robot role.
 */
class StrategyManager {
  private:
	RoleAssignment currentRoles_;

	// ========================================================================
	// STRATEGY SELECTION
	// ========================================================================

	/**
	 * @brief Determine optimal strategy based on game state
	 * @param state Current game state
	 * @return Selected strategy
	 */
	Strategies determineStrategy(const GameState& state) const;

	/**
	 * @brief Assign roles to robots based on strategy
	 * @param strategy Current strategy
	 * @param state Current game state
	 */
	void assignRoles(Strategies strategy, const GameState& state);

	// ========================================================================
	// ROLE BEHAVIORS
	// ========================================================================

	/**
	 * @brief Execute interceptor behavior
	 * Chases ball until possession is achieved
	 */
	void executeInterceptor(Robot& bot, const Robot& teammate, const Ball& ball,
						   const FieldMap& fieldMap, const Robot& rival1,
						   const Robot& rival2, bool isBot1) const;

	/**
	 * @brief Execute defender behavior
	 * Protects own goal with three alert levels
	 */
	void executeDefender(Robot& bot, const Robot& teammate, const Ball& ball,
						const FieldMap& fieldMap) const;

	/**
	 * @brief Execute support behavior
	 * Manages ball distribution when in possession
	 */
	void executeSupport(Robot& bot, const Robot& teammate, const Ball& ball,
					   const FieldMap& fieldMap, const Robot& rival1,
					   const Robot& rival2) const;

	/**
	 * @brief Execute striker behavior
	 * Positions for scoring opportunities
	 */
	void executeStriker(Robot& bot, const Robot& teammate, const Ball& ball,
					   const FieldMap& fieldMap, const Robot& rival1) const;

	/**
	 * @brief Execute attacker behavior
	 * Applies pressure and attempts to score
	 */
	void executeAttacker(Robot& bot, const Robot& teammate, const Ball& ball,
						const FieldMap& fieldMap) const;

	/**
	 * @brief Execute camper behavior
	 * Midfield presence for stuck ball situations
	 */
	void executeCamper(Robot& bot, const Robot& teammate, const Ball& ball) const;

	/**
	 * @brief Execute off-field behavior
	 * Robot not in active play
	 */
	void executeOffField(Robot& bot) const;

  public:
	StrategyManager();

	/**
	 * @brief Update strategy and execute robot behaviors
	 * Main entry point called each frame by GameState
	 * @param state Current game state
	 */
	void update(GameState& state);

	/**
	 * @brief Get current role assignment
	 */
	const RoleAssignment& getCurrentRoles() const {
		return currentRoles_;
	}

	/**
	 * @brief Reset strategy to default state
	 */
	void reset();
};

#endif  // STRATEGY_H
