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

StrategyManager::StrategyManager() : currentRoles_() {
}

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

    if (!ball.isOnField()) {
        return Strategies::CAMPING;
    }

    // Handle off-field scenarios
    if (!bot1Active && !bot2Active)
        return Strategies::NO_STRATEGY;

    if (!bot1Active || !bot2Active)
        return Strategies::OFFIELD_STRATEGY;

    Player robotWithBall = state.whoHasBallControl();
    bool alliesHaveBall = (robotWithBall == HOMEBOT_1 || robotWithBall == HOMEBOT_2);
    bool rivalsHaveBall = (robotWithBall == RIVALBOT_1 || robotWithBall == RIVALBOT_2);

    // Immediate camping in nearest neutral point if ball is stuck
    if (fieldMap.isBallStucked())
        return Strategies::CAMPING;

    float ballX = ball.getPosX();

    // Against one or no opponents - simplified strategy
    if (fieldMap.activeOpponents <= 1) {
        if (!rivalsHaveBall || alliesHaveBall || fieldMap.activeOpponents == 0)
            return Strategies::OFFENSIVE;
        else
            return Strategies::DEFENSIVE;
    }

    // Defend if ball is near our goal and we don't have control
    if (ballX < 0.0f && !alliesHaveBall && rivalsHaveBall) {
        return Strategies::DEFENSIVE;
    }

    // Attack if ball is near opponent's goal and we have control
    if (ballX > 0.0f && alliesHaveBall && !rivalsHaveBall) {
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

    float ballX = ball.getPosX();
    float ballZ = ball.getPosZ();

    float dist1 = bot1.distanceTo(ballX, ballZ);
    float dist2 = bot2.distanceTo(ballX, ballZ);

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
            if (bot1.getPosX() > bot2.getPosX()) {
                currentRoles_.bot1Role = Roles::STRIKER;
                currentRoles_.bot2Role = Roles::SUPPORT;
            } else {
                currentRoles_.bot1Role = Roles::SUPPORT;
                currentRoles_.bot2Role = Roles::STRIKER;
            }
            break;

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

        default:
            std::cerr << "Unknown strategy." << std::endl;
    }
}

// ============================================================================
// ROLE BEHAVIORS - Detailed implementation of each role's actions
// ============================================================================

void StrategyManager::executeInterceptor(Robot& bot, const Ball& ball) const {
    std::cerr << ">> INTERCEPTOR" << std::endl;

    // If ball is acquired, strategy will re-evaluate next cycle
    if (bot.hasBallControl(ball)) {
        std::cerr << "  Ball acquired - strategy will re-evaluate next cycle" << std::endl;
        bot.holdPosition();
        return;
    }

    // Without ball possession - pursue while avoiding collisions
    std::cerr << "  Chasing ball  " << std::endl;

    float targetX = ball.getPosX() + ball.getVelX() * 0.15f;
    float targetZ = ball.getPosZ() + ball.getVelZ() * 0.15f;

    bot.setDribbler(1.0f);
    bot.setKick(0.0f);
    bot.setChip(0.0f);
    bot.faceTowards(ball.getPosX(), ball.getPosZ());

    if (bot.distanceTo(targetX, targetZ) < 0.1f) {
        bot.holdPosition();
        return;
    }
    bot.moveTo(targetX, targetZ);
}

void StrategyManager::executeDefender(Robot& bot,
                                      const Ball& ball,
                                      const FieldMap& fieldMap) const {
    std::cerr << ">> DEFENDER" << std::endl;

    float ballDistToGoal = fieldMap.ballDistToOwnGoal;
    float botDistToBall = bot.distanceTo(ball.getPosX(), ball.getPosZ());

    // Level 1: DANGER - Ball very close to goal
    if (ballDistToGoal < 0.4f) {
        std::cerr << "  DANGER! Ball very close to goal" << std::endl;

        if (botDistToBall < 0.12f) {
            std::cerr << "  Clearing ball away from goal" << std::endl;
            bot.clearBall(ball);
            return;
        } else {
            std::cerr << "  Moving to intercept ball" << std::endl;
            bot.chaseBall(ball);
            return;
        }
    }

    // Level 2: ALERT - Ball in danger zone
    if (ballDistToGoal < 0.6f) {
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

void StrategyManager::executeSupport(GameState& gamestate_g,
                                     Robot& bot,
                                     const Robot& teammate,
                                     const Ball& ball,
                                     const FieldMap& fieldMap,
                                     const Robot& rival1,
                                     const Robot& rival2) const {
    std::cerr << ">> SUPPORT" << std::endl;

    if (!bot.hasBallControl(ball) && !teammate.hasBallControl(ball)) {
        std::cerr << "  No ball possession - positioning for support" << std::endl;

        float supportX = ball.getPosX() - 0.45f;
        float supportZ = ball.getPosZ() + (ball.getPosZ() > 0 ? -0.3f : 0.3f);

        bot.moveTo(supportX, supportZ);
        return;
    }

    std::cerr << "  Ball possessed - searching for STRIKER" << std::endl;

    // Calculate pass quality to striker
    float passQuality = fieldMap.calculatePassQuality(bot, teammate, rival1, rival2);

    std::cerr << "  Pass quality to STRIKER: " << passQuality << std::endl;

    if (passQuality > 0.35f) {
        std::cerr << "  EXECUTING PASS to STRIKER" << std::endl;
        gamestate_g.quickPass();
        return;
    }

    std::cerr << "  Pass not viable - dribbling forward" << std::endl;
    bot.dribbleToGoal();
}

void StrategyManager::executeStriker(Robot& bot, const Robot& teammate, const Ball& ball) const {
    std::cerr << ">> STRIKER" << std::endl;

    if (bot.hasBallControl(ball)) {
        std::cerr << "  Ball possessed - attempting to score" << std::endl;

        float distToGoal = ball.distanceTo(RIGHT_GOAL_X, 0.0f);

        if (distToGoal < 0.6f) {
            std::cerr << "  In shooting range - TAKING SHOT" << std::endl;
            bot.shootAtGoal(ball);
        } else {
            std::cerr << "  Too far from goal - dribbling closer" << std::endl;
            bot.dribbleToGoal();
        }
        return;
    }

    if (bot.getPosX() > teammate.getPosX()) {
        bot.chaseBall(ball);
        return;
    }

    std::cerr << "  Positioning for effective pass reception" << std::endl;

    float ballX = ball.getPosX();
    float ballZ = ball.getPosZ();

    float targetX = std::max(ballX + 0.4f, 0.6f);
    float targetZ = ballZ * 0.2f;

    std::cerr << "  Target position: (" << targetX << ", " << targetZ << ")" << std::endl;
    bot.moveToWhileFacing(targetX, targetZ, ball.getPosX(), ball.getPosZ());
}

void StrategyManager::executeAttacker(Robot& bot, const Robot& teammate, const Ball& ball) const {
    std::cerr << ">> ATTACKER" << std::endl;

    bool weHaveBall = bot.hasBallControl(ball) || teammate.hasBallControl(ball);

    if (weHaveBall) {
        std::cerr << "  Team has possession - pressing forward" << std::endl;

        if (bot.hasBallControl(ball)) {
            std::cerr << "  I have ball - ATTACKING" << std::endl;

            float distToGoal = ball.distanceTo(RIGHT_GOAL_X, 0.0f);

            if (distToGoal < 0.45f) {
                std::cerr << "  Close to goal - SHOOTING" << std::endl;
                bot.shootAtGoal(ball);
            } else {
                std::cerr << "  Dribbling toward goal" << std::endl;
                bot.dribbleToGoal();
            }
        } else {
            std::cerr << "  Teammate has ball - supporting attack" << std::endl;

            float attackX = teammate.getPosX() + 0.25f;
            float attackZ = (teammate.getPosZ() > 0) ? -0.2f : 0.2f;

            bot.moveTo(attackX, attackZ);
            bot.faceTowards(ball.getPosX(), ball.getPosZ());
        }
        return;
    }

    std::cerr << "  No possession - aggressive pressing" << std::endl;

    float ballX = ball.getPosX();
    float ballZ = ball.getPosZ();
    float distToBall = bot.distanceTo(ballX, ballZ);

    if (distToBall < 0.4f) {
        std::cerr << "  Close to ball - chasing directly" << std::endl;
        bot.chaseBall(ball);
        return;
    }

    float pressX = std::max(ballX, 0.0f) + 0.2f;
    float pressZ = ball.getPosZ() * 0.7f;

    std::cerr << "  Pressing position: (" << pressX << ", " << pressZ << ")" << std::endl;
    bot.moveToWhileFacing(pressX, pressZ, ball.getPosX(), ball.getPosZ());
}

void StrategyManager::executeCamper(Robot& bot) const {
    std::cerr << ">> CAMPER" << std::endl;

    float campX = -0.15f;
    float campZ = 0;

    std::cerr << "  Maintaining camp position" << std::endl;

    bot.moveToWhileFacing(campX, campZ, 0.0f, 0.0f);
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
        case Strategies::CAMPING:
            std::cerr << "DEF_CAMPING";
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
            executeDefender(bot1, ball, fieldMap);
            break;
        case Roles::INTERCEPTOR:
            executeInterceptor(bot1, ball);
            break;
        case Roles::CAMPER:
            executeCamper(bot1);
            break;
        case Roles::SUPPORT:
            executeSupport(state, bot1, bot2, ball, fieldMap, rival1, rival2);
            break;
        case Roles::ATTACKER:
            executeAttacker(bot1, bot2, ball);
            break;
        case Roles::STRIKER:
            executeStriker(bot1, bot2, ball);
            break;
        case Roles::OFFIELD:
            executeOffField(bot1);
            break;
    }

    // Step 4: Execute behavior for bot2 based on assigned role
    switch (currentRoles_.bot2Role) {
        case Roles::DEFENDER:
            executeDefender(bot2, ball, fieldMap);
            break;
        case Roles::INTERCEPTOR:
            executeInterceptor(bot2, ball);
            break;
        case Roles::CAMPER:
            executeCamper(bot2);
            break;
        case Roles::SUPPORT:
            executeSupport(state, bot2, bot1, ball, fieldMap, rival1, rival2);
            break;
        case Roles::ATTACKER:
            executeAttacker(bot2, bot1, ball);
            break;
        case Roles::STRIKER:
            executeStriker(bot2, bot1, ball);
            break;
        case Roles::OFFIELD:
            executeOffField(bot2);
            break;
    }
}
