/**
 * @file strategy.cpp
 * @brief Strategy management and role assignment implementation
 * @author Agustin Valenzuela, Alex Petersen, Dylan Frigerio, Enzo Fernandez Rosas
 * @copyright Copyright (c) 2025
 */

#include "strategy.h"

#include <algorithm>
#include <cmath>
#include <iostream>

#include "constants.h"
#include "gamestate.h"
#include "geometry.h"

// ============================================================================
// CONSTRUCTOR
// ============================================================================

StrategyManager::StrategyManager() : currentRoles_() {}

void StrategyManager::reset() {
    currentRoles_ = RoleAssignment();
}

// ============================================================================
// STRATEGY DETERMINATION
// ============================================================================

Strategies StrategyManager::determineStrategy(const GameState& state) const {
    const Robot& bot1 = state.getHomeBot1();
    const Robot& bot2 = state.getHomeBot2();
    const Ball& ball = state.getBall();
    const FieldMap& fieldMap = state.getFieldMap();

    // Check field status
    bool bot1Active = bot1.isOnField();
    bool bot2Active = bot2.isOnField();

    // Handle off-field scenarios
    if (!bot1Active && !bot2Active)
        return Strategies::NO_STRATEGY;

    if (!bot1Active || !bot2Active)
        return Strategies::OFFIELD_STRATEGY;

    bool bot1HasBall = bot1.hasBallControl(ball);
    bool bot2HasBall = bot2.hasBallControl(ball);
    bool anyBotHasBall = bot1HasBall || bot2HasBall;

    // Against one or no opponents - simplified strategy
    if (fieldMap.activeOpponents <= 1) {
        if (anyBotHasBall)
            return Strategies::OFFENSIVE;
        else
            return Strategies::DEFENSIVE;
    }

    // Priority 1: Immediate camping in nearest neutral point if ball is stuck
    if (fieldMap.isBallStuck())
		return Strategies::CAMPING;

    float ballX = ball.getPosX();

    // Priority 2: Defend if ball is near our goal and we don't have control
    if (ballX < 0.0f && !anyBotHasBall) {
        return Strategies::DEFENSIVE;
    }

    // Priority 3: Attack if ball is near opponent's goal and we have control
    if (ballX > 0.0f && anyBotHasBall) {
        return Strategies::OFFENSIVE;
    }

    // Default: one attacker, one defender
    return Strategies::BALANCED;
}

// ============================================================================
// ROLE ASSIGNMENT
// ============================================================================

void StrategyManager::assignRoles(Strategies strategy, const GameState& state) {
    const Robot& bot1 = state.getHomeBot1();
    const Robot& bot2 = state.getHomeBot2();
    const Ball& ball = state.getBall();

    bool bot1Active = bot1.isOnField();
    bool bot2Active = bot2.isOnField();

    // Calculate distances to ball
    float ballX = ball.getPosX();
	float ballZ = ball.getPosZ();
    float dist1 = bot1.distanceTo(ballX, ballZ);
    float dist2 = bot2.distanceTo(ballX, ballZ);

    bool bot1HasBall = bot1.hasBallControl(ball);
    bool bot2HasBall = bot2.hasBallControl(ball);
    bool bot1Closer = dist1 < dist2;

    // Assign roles based on strategy
    switch (strategy) {
        case Strategies::NO_STRATEGY:
            currentRoles_.bot1Role = Roles::OFFIELD;
            currentRoles_.bot2Role = Roles::OFFIELD;
            break;

        case Strategies::OFFIELD_STRATEGY:
            // Active bot is defender, inactive is off-field
            if (bot1Active) {
                currentRoles_.bot1Role = Roles::DEFENDER;
                currentRoles_.bot2Role = Roles::OFFIELD;
            } else {
                currentRoles_.bot1Role = Roles::OFFIELD;
                currentRoles_.bot2Role = Roles::DEFENDER;
            }
            break;

        case Strategies::DEFENSIVE:
            // One defender, one interceptor
            if (bot1Closer) {
                currentRoles_.bot1Role = Roles::INTERCEPTOR;
                currentRoles_.bot2Role = Roles::DEFENDER;
            } else {
                currentRoles_.bot1Role = Roles::DEFENDER;
                currentRoles_.bot2Role = Roles::INTERCEPTOR;
            }
            break;

        case Strategies::CAMPING:
            // One defender, one camper
            if (bot1Closer) {
                currentRoles_.bot1Role = Roles::INTERCEPTOR;
                currentRoles_.bot2Role = Roles::CAMPER;
            } else {
                currentRoles_.bot1Role = Roles::CAMPER;
                currentRoles_.bot2Role = Roles::INTERCEPTOR;
            }
            break;

        case Strategies::OFFENSIVE:
            // One support, one striker (priority to ball holder)
            if (bot1HasBall || (bot1Closer && !bot2HasBall)) {
                currentRoles_.bot1Role = Roles::STRIKER;
                currentRoles_.bot2Role = Roles::SUPPORT;
            } else {
                currentRoles_.bot1Role = Roles::SUPPORT;
                currentRoles_.bot2Role = Roles::STRIKER;
            }
            break;
        default:
            std::cerr << "Unknown strategy, defaults to BALANCED." << std::endl;
        case Strategies::BALANCED:
            // One defender, one attacker
            if (bot1Closer) {
                currentRoles_.bot1Role = Roles::ATTACKER;
                currentRoles_.bot2Role = Roles::DEFENDER;
            } else {
                currentRoles_.bot1Role = Roles::DEFENDER;
                currentRoles_.bot2Role = Roles::ATTACKER;
            }
            break;
    }
}

// ============================================================================
// ROLE BEHAVIORS - Detailed implementation of each role's actions
// ============================================================================

void StrategyManager::executeInterceptor(Robot& bot, const Robot& teammate, const Ball& ball,
                                        const FieldMap& fieldMap, const Robot& rival1,
                                        const Robot& rival2, bool isBot1) const {
    std::cerr << ">> INTERCEPTOR" << std::endl;

    // If ball is acquired, strategy will re-evaluate next cycle
    if (bot.hasBallControl(ball)) {
        std::cerr << "  Ball acquired - strategy will re-evaluate next cycle" << std::endl;
        bot.holdPosition();
        return;
    }

    // Without ball possession - pursue while avoiding collisions
    std::cerr << "  Chasing ball - collision avoidance active" << std::endl;

    float targetX = fieldMap.predictedBallX;
    float targetZ = fieldMap.predictedBallZ;

    // Avoid collision with teammate
    float distToTeammate = bot.distanceTo(teammate);
    if (distToTeammate < 0.16f) {
        std::cerr << "  Too close to teammate - adjusting path" << std::endl;
        // Adjust position laterally to avoid collision

        targetZ += (bot.getPosZ() > teammate.getPosZ()) ? 0.15f : -0.15f;
    }

    // Avoid collision with opponents
    if (rival1.isOnField()) {
        float distToRival1 = bot.distanceTo(rival1);
        if (distToRival1 < 0.3f) {
            std::cerr << "  Avoiding opponent collision" << std::endl;
            targetX += 0.15f;  // Adjust forward to go around
        }
    }

    bot.moveTo(targetX, targetZ);
}

void StrategyManager::executeDefender(Robot& bot, const Robot& teammate, const Ball& ball,
                                     const FieldMap& fieldMap) const {
    std::cerr << ">> DEFENDER" << std::endl;

    float ballDistToGoal = fieldMap.ballDistToOwnGoal;
    float botDistToBall = bot.distanceTo(ball);

    // Level 1: DANGER - Ball very close to goal
    if (ballDistToGoal < 0.5f) {
        std::cerr << "  DANGER! Ball very close to goal" << std::endl;

        if (botDistToBall < 0.35f) {
            if (bot.hasBallControl(ball)) {
                std::cerr << "  Ball controlled - maintaining possession" << std::endl;
                bot.holdPosition();
                return;
            } else {
                std::cerr << "  Clearing ball away from goal" << std::endl;
                bot.clearBall(ball);
                return;
            }
        } else {
            std::cerr << "  Moving to intercept ball" << std::endl;
            bot.chaseBall(ball);
            return;
        }
    }

    // Level 2: ALERT - Ball in danger zone
    if (ballDistToGoal < 0.8f) {
        std::cerr << "  Ball in danger zone - taking defensive position" << std::endl;

        float defX = (ball.getPosX() + LEFT_GOAL_X) * 0.6f;
        float defZ = ball.getPosZ() * 0.5f;

        defX = std::max(defX, LEFT_GOAL_X + 0.35f);
        defZ = std::clamp(defZ, -0.4f, 0.4f);

        bot.moveToWhileFacing(defX, defZ, ball.getPosX(), ball.getPosZ());
        return;
    }

    // Level 3: NORMAL - Ball far away
    std::cerr << "  Normal defensive position - guarding goal" << std::endl;

    float guardX = LEFT_GOAL_X + 0.45f;
    float guardZ = ball.getPosZ() * 0.3f;
    guardZ = std::clamp(guardZ, -0.35f, 0.35f);

    bot.moveToWhileFacing(guardX, guardZ, ball.getPosX(), ball.getPosZ());
}

void StrategyManager::executeSupport(Robot& bot, const Robot& teammate, const Ball& ball,
                                    const FieldMap& fieldMap, const Robot& rival1,
                                    const Robot& rival2) const {
    std::cerr << ">> SUPPORT" << std::endl;

    if (!bot.hasBallControl(ball)) {
        std::cerr << "  No ball possession - positioning for support" << std::endl;

        float supportX = ball.getPosX() - 0.3f;
        float supportZ = ball.getPosZ() + (ball.getPosZ() > 0 ? -0.4f : 0.4f);

        supportX = std::clamp(supportX, -FIELD_HALF_LENGTH + 0.3f, FIELD_HALF_LENGTH - 0.5f);
        supportZ = std::clamp(supportZ, -FIELD_HALF_WIDTH + 0.3f, FIELD_HALF_WIDTH - 0.3f);

        bot.moveTo(supportX, supportZ);
        return;
    }

    std::cerr << "  Ball possessed - searching for STRIKER" << std::endl;

    if (!teammate.isOnField()) {
        std::cerr << "  No teammate available - advancing with ball" << std::endl;
        bot.dribbleToGoal();
        return;
    }

    // Calculate pass quality to striker
    float passQuality = fieldMap.calculatePassQuality(bot, teammate, rival1, rival2);

    std::cerr << "  Pass quality to STRIKER: " << passQuality << std::endl;

    if (passQuality > 0.35f) {
        std::cerr << "  EXECUTING PASS to STRIKER" << std::endl;

        float dx = teammate.getPosX() - bot.getPosX();
        float dz = teammate.getPosZ() - bot.getPosZ();
        float angle = std::atan2(dz, dx);

        bot.setTargetAngle(angle);

        float dist = bot.distanceTo(teammate);
        float kickPower = std::min(1.0f, dist * 0.8f);
        bot.setKickX(kickPower);

        return;
    }

    std::cerr << "  Pass not viable - dribbling forward" << std::endl;
    bot.dribbleToGoal();
}

void StrategyManager::executeStriker(Robot& bot, const Robot& teammate, const Ball& ball,
                                    const FieldMap& fieldMap, const Robot& rival1) const {
    std::cerr << ">> STRIKER" << std::endl;

    if (bot.hasBallControl(ball)) {
        std::cerr << "  Ball possessed - attempting to score" << std::endl;

        float distToGoal = ball.distanceTo(RIGHT_GOAL_X, 0.0f);

        if (distToGoal < 0.7f) {
            std::cerr << "  In shooting range - TAKING SHOT" << std::endl;
            bot.shootAtGoal(ball);
        } else {
            std::cerr << "  Too far from goal - dribbling closer" << std::endl;
            bot.dribbleToGoal();
        }
        return;
    }

    std::cerr << "  Positioning for effective pass reception" << std::endl;

    float ballX = ball.getPosX();
    float ballZ = ball.getPosZ();

    float idealX = std::max(ballX + 0.5f, 0.3f);
    float idealZ = 0.0f;

    // Avoid opponent penalty area
    if (fieldMap.inOppPenaltyArea(idealX, idealZ)) {
        idealX = fieldMap.oppPenaltyMinX - 0.15f;
    }

    // Adjust position based on opponent locations
    if (rival1.isOnField()) {
        float rival1X = rival1.getPosX();
        float rival1Z = rival1.getPosZ();

        if (std::abs(rival1X - idealX) < 0.3f && std::abs(rival1Z) < 0.3f) {
            idealZ = (ballZ > 0) ? 0.4f : -0.4f;
        }
    }

    idealX = std::clamp(idealX, -FIELD_HALF_LENGTH + 0.5f, FIELD_HALF_LENGTH - 0.3f);
    idealZ = std::clamp(idealZ, -FIELD_HALF_WIDTH + 0.3f, FIELD_HALF_WIDTH - 0.3f);

    std::cerr << "  Target position: (" << idealX << ", " << idealZ << ")" << std::endl;
    bot.moveToWhileFacing(idealX, idealZ, ball.getPosX(), ball.getPosZ());
}

void StrategyManager::executeAttacker(Robot& bot, const Robot& teammate, const Ball& ball,
                                     const FieldMap& fieldMap) const {
    std::cerr << ">> ATTACKER" << std::endl;

    bool weHaveBall = bot.hasBallControl(ball) || teammate.hasBallControl(ball);

    if (weHaveBall) {
        std::cerr << "  Team has possession - pressing forward" << std::endl;

        if (bot.hasBallControl(ball)) {
            std::cerr << "  I have ball - ATTACKING" << std::endl;

            float distToGoal = ball.distanceTo(RIGHT_GOAL_X, 0.0f);

            if (distToGoal < 0.6f) {
                std::cerr << "  Close to goal - SHOOTING" << std::endl;
                bot.shootAtGoal(ball);
            } else {
                std::cerr << "  Dribbling toward goal" << std::endl;
                bot.dribbleToGoal();
            }
        } else {
            std::cerr << "  Teammate has ball - supporting attack" << std::endl;

            float attackX = teammate.getPosX() + 0.4f;
            float attackZ = (teammate.getPosZ() > 0) ? -0.3f : 0.3f;

            attackX = std::clamp(attackX, 0.0f, FIELD_HALF_LENGTH - 0.3f);
            attackZ = std::clamp(attackZ, -FIELD_HALF_WIDTH + 0.3f, FIELD_HALF_WIDTH - 0.3f);

            bot.moveTo(attackX, attackZ);
        }
        return;
    }

    std::cerr << "  No possession - aggressive pressing" << std::endl;

    float ballX = ball.getPosX();
    float distToBall = bot.distanceTo(ball);

    if (distToBall < 0.5f) {
        std::cerr << "  Close to ball - chasing directly" << std::endl;
        bot.chaseBall(ball);
        return;
    }

    float pressX = std::max(ballX, 0.0f) + 0.2f;
    float pressZ = ball.getPosZ() * 0.7f;

    pressX = std::clamp(pressX, 0.0f, FIELD_HALF_LENGTH - 0.3f);
    pressZ = std::clamp(pressZ, -FIELD_HALF_WIDTH + 0.3f, FIELD_HALF_WIDTH - 0.3f);

    std::cerr << "  Pressing position: (" << pressX << ", " << pressZ << ")" << std::endl;
    bot.moveToWhileFacing(pressX, pressZ, ball.getPosX(), ball.getPosZ());
}

void StrategyManager::executeCamper(Robot& bot, const Robot& teammate, const Ball& ball) const {
    std::cerr << ">> CAMPER" << std::endl;

    float campX = ball.getPosX() * 0.7f;
    float campZ = ball.getPosZ() * 0.5f;

    float distToBall = bot.distanceTo(ball);

    if (distToBall < 0.35f) {
        std::cerr << "  Ball nearby - engaging" << std::endl;
        bot.chaseBall(ball);
        return;
    }

    std::cerr << "  Maintaining camp position" << std::endl;

    campX = std::clamp(campX, -0.6f, 0.6f);
    campZ = std::clamp(campZ, -FIELD_HALF_WIDTH + 0.3f, FIELD_HALF_WIDTH - 0.3f);

    bot.moveToWhileFacing(campX, campZ, ball.getPosX(), ball.getPosZ());
}

void StrategyManager::executeOffField(Robot& bot) const {
    std::cerr << ">> OFFIELD - Maintaining position" << std::endl;
    bot.holdPosition();
}

// ============================================================================
// MAIN UPDATE
// ============================================================================

void StrategyManager::update(GameState& state) {
    // Step 1: Select optimal strategy based on current game state
    Strategies selectedStrategy = determineStrategy(state);
    currentRoles_.currentStrategy = selectedStrategy;

    // Step 2: Assign specific roles to each robot based on selected strategy
    assignRoles(selectedStrategy, state);

    // Log current strategy for debugging
    std::cerr << "\n[STRATEGY: ";
    switch (selectedStrategy) {
        case Strategies::DEFENSIVE:
            std::cerr << "DEFENSIVE";
            break;
        case Strategies::BALANCED:
            std::cerr << "BALANCED";
            break;
        case Strategies::OFFENSIVE:
            std::cerr << "OFFENSIVE";
            break;
        case Strategies::DEFENSIVE_CAMPING:
            std::cerr << "DEF_CAMPING";
            break;
        case Strategies::OFFENSIVE_CAMPING:
            std::cerr << "OFF_CAMPING";
            break;
        case Strategies::OFFIELD_STRATEGY:
            std::cerr << "OFFIELD";
            break;
        case Strategies::NO_STRATEGY:
            std::cerr << "NO_STRATEGY";
            break;
    }
    std::cerr << "]" << std::endl;

    // Get references for easier access
    Robot& bot1 = state.getHomeBot1();
    Robot& bot2 = state.getHomeBot2();
    const Ball& ball = state.getBall();
    const FieldMap& fieldMap = state.getFieldMap();
    const Robot& rival1 = state.getRivalBot1();
    const Robot& rival2 = state.getRivalBot2();

    // Step 3: Execute behavior for bot1 based on assigned role
    switch (currentRoles_.bot1Role) {
        case Roles::DEFENDER:
            executeDefender(bot1, bot2, ball, fieldMap);
            break;
        case Roles::INTERCEPTOR:
            executeInterceptor(bot1, bot2, ball, fieldMap, rival1, rival2, true);
            break;
        case Roles::CAMPER:
            executeCamper(bot1, bot2, ball);
            break;
        case Roles::SUPPORT:
            executeSupport(bot1, bot2, ball, fieldMap, rival1, rival2);
            break;
        case Roles::ATTACKER:
            executeAttacker(bot1, bot2, ball, fieldMap);
            break;
        case Roles::STRIKER:
            executeStriker(bot1, bot2, ball, fieldMap, rival1);
            break;
        case Roles::OFFIELD:
            executeOffField(bot1);
            break;
    }

    // Step 4: Execute behavior for bot2 based on assigned role
    switch (currentRoles_.bot2Role) {
        case Roles::DEFENDER:
            executeDefender(bot2, bot1, ball, fieldMap);
            break;
        case Roles::INTERCEPTOR:
            executeInterceptor(bot2, bot1, ball, fieldMap, rival1, rival2, false);
            break;
        case Roles::CAMPER:
            executeCamper(bot2, bot1, ball);
            break;
        case Roles::SUPPORT:
            executeSupport(bot2, bot1, ball, fieldMap, rival1, rival2);
            break;
        case Roles::ATTACKER:
            executeAttacker(bot2, bot1, ball, fieldMap);
            break;
        case Roles::STRIKER:
            executeStriker(bot2, bot1, ball, fieldMap, rival1);
            break;
        case Roles::OFFIELD:
            executeOffField(bot2);
            break;
    }
}
