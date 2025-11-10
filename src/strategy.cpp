#include "strategy.h"

#include <algorithm>
#include <cmath>
#include <iostream>

#include "gamestate.h"
#include "geometry.h"
#include "robot.h"

void manageStrategy(const gameState& state);

/**
 * @brief Manages the strategy to follow based on the state of the game.
 * @param botRivalsCount Number of rival bots.
 */
Strategies determineStrategy() {
    // Check field status
    bool bot1Active = playerList_[HOMEBOT_1].isOnField();
    bool bot2Active = playerList_[HOMEBOT_2].isOnField();

    // Handle off-field scenarios
    if (!bot1Active && !bot2Active)
        return Strategies::NO_STRATEGY;

    if (!bot1Active || !bot2Active)
        return Strategies::OFFIELD_STRATEGY;

    bool bot1HasBall = playerList_[HOMEBOT_1].hasBallControl(ball_);
    bool bot2HasBall = playerList_[HOMEBOT_2].hasBallControl(ball_);
    bool anyBotHasBall = bot1HasBall || bot2HasBall;

    if (FieldMap.activeOpponents <= 1) {
        if (anyBotHasBall)
            return Strategies::OFFENSIVE;
        else
            return Strategies::DEFENSIVE;
    }

    // Analyze game situation
    bool ballStuck = fieldMap_.isBallStuck();
    float ballX = ball_.getPosX();
    float ballZ = ball_.getPosZ();

    float ballDistToOwnGoal = fieldMap_.ballDistToOwnGoal;
    float ballDistToOppGoal = fieldMap_.ballDistToOppGoal;

    // Priority 1: DEFENSIVE - Ball near own goal
    if (ballDistToOwnGoal < 0.7f && !anyBotHasBall) {
        return Strategies::DEFENSIVE;
    }

    // Priority 2: OFFENSIVE - Ball near opponent goal or we have control
    if (ballDistToOppGoal < 0.8f || anyBotHasBall) {
        return Strategies::OFFENSIVE;
    }

    // Priority 3: CAMPING strategies when ball is stuck
    if (ballStuck) {
        if (ballX > 0.5f) {
            return Strategies::OFFENSIVE_CAMPING;
        } else if (ballX > -0.5f && ballX < 0.5f) {
            return Strategies::DEFENSIVE_CAMPING;
        }
    }

    // Default: BALANCED
    return Strategies::BALANCED;
}

void GameState::assignRoles(Strategies strategy) {
    bool bot1Active = playerList_[HOMEBOT_1].isOnField();
    bool bot2Active = playerList_[HOMEBOT_2].isOnField();

    // Handle off-field scenarios
    if (!bot1Active && !bot2Active) {
        currentRoles_.bot1Role = Roles::OFFIELD;
        currentRoles_.bot2Role = Roles::OFFIELD;
        return;
    }

    if (!bot1Active) {
        currentRoles_.bot1Role = Roles::OFFIELD;
        currentRoles_.bot2Role = Roles::DEFENDER;
        return;
    }

    if (!bot2Active) {
        currentRoles_.bot1Role = Roles::DEFENDER;
        currentRoles_.bot2Role = Roles::OFFIELD;
        return;
    }

    // Calculate distances to ball
    float ballX = ball_.getPosX();
    float dist1 = playerList_[HOMEBOT_1].distanceTo(Robot(ballX, 0, ball_.getPosZ()));
    float dist2 = playerList_[HOMEBOT_2].distanceTo(Robot(ballX, 0, ball_.getPosZ()));

    bool bot1HasBall = playerList_[HOMEBOT_1].hasBallControl(ball_);
    bool bot2HasBall = playerList_[HOMEBOT_2].hasBallControl(ball_);
    bool bot1Closer = dist1 < dist2;

    // Assign roles based on strategy
    switch (strategy) {
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

        case Strategies::OFFENSIVE_CAMPING:
            // One camper, one attacker
            if (bot1Closer) {
                currentRoles_.bot1Role = Roles::CAMPER;
                currentRoles_.bot2Role = Roles::ATTACKER;
            } else {
                currentRoles_.bot1Role = Roles::ATTACKER;
                currentRoles_.bot2Role = Roles::CAMPER;
            }
            break;

        case Strategies::DEFENSIVE_CAMPING:
            // One defender, one camper
            if (bot1Closer) {
                currentRoles_.bot1Role = Roles::CAMPER;
                currentRoles_.bot2Role = Roles::DEFENDER;
            } else {
                currentRoles_.bot1Role = Roles::DEFENDER;
                currentRoles_.bot2Role = Roles::CAMPER;
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

        case Strategies::OFFIELD_STRATEGY:
        default:
            // Already handled above
            break;
    }
}

// ============================================================================
// ROLE BEHAVIORS - Detailed implementation of each role's actions
// ============================================================================

/**
 * INTERCEPTOR: Chases ball until possession is achieved
 * - Aggressively pursues the ball
 * - Avoids collisions with teammates and opponents (only when we don't have ball)
 * - Re-evaluates entire strategy once ball is acquired
 */
void GameState::executeInterceptor(Robot& bot, const Robot& teammate, bool isBot1) {
    std::cerr << ">> INTERCEPTOR" << std::endl;

    // If ball is acquired, trigger complete strategy re-evaluation
    if (bot.hasBallControl(ball_)) {
        std::cerr << "  Ball acquired - RE-EVALUATING STRATEGY" << std::endl;

        // Force strategy re-evaluation with new game state
        Strategies newStrategy = selectStrategy();
        currentRoles_.currentStrategy = newStrategy;
        assignRoles(newStrategy);

        std::cerr << "  Strategy changed, new role will execute next cycle" << std::endl;
        return; // New role will be executed in next cycle
    }

    // Without ball possession - pursue while avoiding collisions
    std::cerr << "  Chasing ball - collision avoidance active" << std::endl;

    float targetX = fieldMap_.predictedBallX;
    float targetZ = fieldMap_.predictedBallZ;

    // Avoid collision with teammate (only when we don't have the ball)
    float distToTeammate = bot.distanceTo(teammate);
    if (distToTeammate < 0.25f) {
        std::cerr << "  Too close to teammate - adjusting path" << std::endl;
        // Adjust position laterally to avoid collision
        targetZ += (bot.getPosZ() > teammate.getPosZ()) ? 0.15f : -0.15f;
        targetZ = std::clamp(targetZ, -FIELD_HALF_WIDTH + 0.2f, FIELD_HALF_WIDTH - 0.2f);
    }

    // Avoid collision with opponents
    if (playerList_[RIVALBOT_1].isOnField()) {
        float distToRival1 = bot.distanceTo(playerList_[RIVALBOT_1]);
        if (distToRival1 < 0.3f) {
            std::cerr << "  Avoiding opponent collision" << std::endl;
            targetX += 0.15f; // Adjust forward to go around
        }
    }

    bot.moveTo(targetX, targetZ);
}

/**
 * DEFENDER: Protects own goal and waits for ball arrival
 * - Stays near goal in defensive position
 * - Three action levels based on ball proximity:
 *   1. DANGER (<0.5m): Attempts possession or clears ball
 *   2. ALERT (<0.8m): Positions between ball and goal
 *   3. NORMAL (>0.8m): Maintains defensive position near goal
 */
void GameState::executeDefender(Robot& bot, const Robot& teammate) {
    std::cerr << ">> DEFENDER" << std::endl;

    float ballDistToGoal = fieldMap_.ballDistToOwnGoal;
    float botDistToBall = bot.distanceTo(ball_);

    // Level 1: DANGER - Ball very close to goal, immediate action required
    if (ballDistToGoal < 0.5f) {
        std::cerr << "  DANGER! Ball very close to goal" << std::endl;

        if (botDistToBall < 0.35f) {
            // Close enough to interact with ball
            if (bot.hasBallControl(ball_)) {
                std::cerr << "  Ball controlled - maintaining possession" << std::endl;
                bot.holdPosition();
                return;
            } else {
                std::cerr << "  Clearing ball away from goal" << std::endl;
                bot.clearBall(ball_);
                return;
            }
        } else {
            std::cerr << "  Moving to intercept ball" << std::endl;
            bot.chaseBall(ball_);
            return;
        }
    }

    // Level 2: ALERT - Ball in danger zone, prepare defensive position
    if (ballDistToGoal < 0.8f) {
        std::cerr << "  Ball in danger zone - taking defensive position" << std::endl;

        // Position between ball and goal
        float defX = (ball_.getPosX() + LEFT_GOAL_X) * 0.6f;
        float defZ = ball_.getPosZ() * 0.5f;

        // Ensure minimum distance from goal
        defX = std::max(defX, LEFT_GOAL_X + 0.35f);
        defZ = std::clamp(defZ, -0.4f, 0.4f);

        bot.moveToWhileFacing(defX, defZ, ball_.getPosX(), ball_.getPosZ());
        return;
    }

    // Level 3: NORMAL - Ball far away, maintain guard position near goal
    std::cerr << "  Normal defensive position - guarding goal" << std::endl;

    float guardX = LEFT_GOAL_X + 0.45f;
    float guardZ = ball_.getPosZ() * 0.3f; // Slight lateral adjustment based on ball
    guardZ = std::clamp(guardZ, -0.35f, 0.35f);

    bot.moveToWhileFacing(guardX, guardZ, ball_.getPosX(), ball_.getPosZ());
}

/**
 * SUPPORT: Manages ball distribution when in possession
 * - ONLY acts when it has the ball
 * - Primary objective: Find and execute pass to STRIKER
 * - Calculates pass quality and adjusts kick power based on distance
 * - Falls back to dribbling if no clear passing lane exists
 */
void GameState::executeSupport(Robot& bot, const Robot& teammate) {
    std::cerr << ">> SUPPORT" << std::endl;

    // Support role only activates when ball is possessed
    if (!bot.hasBallControl(ball_)) {
        std::cerr << "  No ball possession - positioning for support" << std::endl;

        // Position to receive pass or provide backup support
        float supportX = ball_.getPosX() - 0.3f;
        float supportZ = ball_.getPosZ() + (ball_.getPosZ() > 0 ? -0.4f : 0.4f);

        supportX = std::clamp(supportX, -FIELD_HALF_LENGTH + 0.3f, FIELD_HALF_LENGTH - 0.5f);
        supportZ = std::clamp(supportZ, -FIELD_HALF_WIDTH + 0.3f, FIELD_HALF_WIDTH - 0.3f);

        bot.moveTo(supportX, supportZ);
        return;
    }

    // Has ball possession - look for STRIKER to pass
    std::cerr << "  Ball possessed - searching for STRIKER" << std::endl;

    if (!teammate.isOnField()) {
        std::cerr << "  No teammate available - advancing with ball" << std::endl;
        bot.dribbleToGoal();
        return;
    }

    // Calculate pass quality to striker
    float passQuality = fieldMap_.calculatePassQuality(
        bot, teammate, playerList_[RIVALBOT_1], playerList_[RIVALBOT_2]);

    std::cerr << "  Pass quality to STRIKER: " << passQuality << std::endl;

    if (passQuality > 0.35f) {
        std::cerr << "  EXECUTING PASS to STRIKER" << std::endl;

        // Align towards striker for accurate pass
        float dx = teammate.getPosX() - bot.getPosX();
        float dz = teammate.getPosZ() - bot.getPosZ();
        float angle = std::atan2(dz, dx);

        bot.setTargetAngle(angle);

        // Adjust kick power based on distance
        float dist = bot.distanceTo(teammate);
        float kickPower = std::min(1.0f, dist * 0.8f);
        bot.setKickX(kickPower);

        return;
    }

    std::cerr << "  Pass not viable - dribbling forward" << std::endl;
    bot.dribbleToGoal();
}

/**
 * STRIKER: Positions for scoring opportunities
 * - When without ball: Positions strategically to receive effective passes
 * - Finds advanced positions while maintaining clear passing lanes
 * - Avoids opponent penalty area
 * - Adjusts position based on opponent locations
 * - When with ball: Attempts to score or advances closer to goal
 */
void GameState::executeStriker(Robot& bot, const Robot& teammate) {
    std::cerr << ">> STRIKER" << std::endl;

    // If ball is possessed - attempt to score
    if (bot.hasBallControl(ball_)) {
        std::cerr << "  Ball possessed - attempting to score" << std::endl;

        float distToGoal = ball_.distanceTo(RIGHT_GOAL_X, 0.0f);

        if (distToGoal < 0.7f) {
            std::cerr << "  In shooting range - TAKING SHOT" << std::endl;
            bot.shootAtGoal(ball_);
        } else {
            std::cerr << "  Too far from goal - dribbling closer" << std::endl;
            bot.dribbleToGoal();
        }
        return;
    }

    // Without ball - position for effective pass reception
    std::cerr << "  Positioning for effective pass reception" << std::endl;

    float ballX = ball_.getPosX();
    float ballZ = ball_.getPosZ();

    // Find advanced position accessible for passing
    float idealX = std::max(ballX + 0.5f, 0.3f); // Forward of ball, minimum midfield
    float idealZ = 0.0f; // Prefer center position

    // Avoid opponent penalty area (illegal positioning)
    if (fieldMap_.inOppPenaltyArea(idealX, idealZ)) {
        idealX = fieldMap_.oppPenaltyMinX - 0.15f;
    }

    // Adjust position based on opponent locations to maintain clear passing lane
    if (playerList_[RIVALBOT_1].isOnField()) {
        float rival1X = playerList_[RIVALBOT_1].getPosX();
        float rival1Z = playerList_[RIVALBOT_1].getPosZ();

        // If opponent blocks center position, move to flank
        if (std::abs(rival1X - idealX) < 0.3f && std::abs(rival1Z) < 0.3f) {
            idealZ = (ballZ > 0) ? 0.4f : -0.4f;
        }
    }

    // Clamp to valid field boundaries
    idealX = std::clamp(idealX, -FIELD_HALF_LENGTH + 0.5f, FIELD_HALF_LENGTH - 0.3f);
    idealZ = std::clamp(idealZ, -FIELD_HALF_WIDTH + 0.3f, FIELD_HALF_WIDTH - 0.3f);

    std::cerr << "  Target position: (" << idealX << ", " << idealZ << ")" << std::endl;
    bot.moveToWhileFacing(idealX, idealZ, ball_.getPosX(), ball_.getPosZ());
}

/**
 * ATTACKER: Applies pressure and attempts to score
 * - Two modes based on ball possession:
 *   1. WE HAVE BALL: Presses forward, supports attack, attempts to score
 *   2. NOBODY HAS BALL: Aggressive pressing, tries to score when opportunity arises
 * - Differentiates between having ball personally vs teammate having it
 */
void GameState::executeAttacker(Robot& bot, const Robot& teammate) {
    std::cerr << ">> ATTACKER" << std::endl;

    bool weHaveBall = bot.hasBallControl(ball_) || teammate.hasBallControl(ball_);

    // Mode 1: WE HAVE BALL - Press forward and support attack
    if (weHaveBall) {
        std::cerr << "  Team has possession - pressing forward" << std::endl;

        if (bot.hasBallControl(ball_)) {
            std::cerr << "  I have ball - ATTACKING" << std::endl;

            float distToGoal = ball_.distanceTo(RIGHT_GOAL_X, 0.0f);

            if (distToGoal < 0.6f) {
                std::cerr << "  Close to goal - SHOOTING" << std::endl;
                bot.shootAtGoal(ball_);
            } else {
                std::cerr << "  Dribbling toward goal" << std::endl;
                bot.dribbleToGoal();
            }
        } else {
            std::cerr << "  Teammate has ball - supporting attack" << std::endl;

            // Position forward to receive or continue pressure
            float attackX = teammate.getPosX() + 0.4f;
            float attackZ = (teammate.getPosZ() > 0) ? -0.3f : 0.3f;

            attackX = std::clamp(attackX, 0.0f, FIELD_HALF_LENGTH - 0.3f);
            attackZ = std::clamp(attackZ, -FIELD_HALF_WIDTH + 0.3f, FIELD_HALF_WIDTH - 0.3f);

            bot.moveTo(attackX, attackZ);
        }
        return;
    }

    // Mode 2: NOBODY HAS BALL - Aggressive pressing and goal attempts
    std::cerr << "  No possession - aggressive pressing" << std::endl;

    float ballX = ball_.getPosX();
    float distToBall = bot.distanceTo(ball_);

    // If close to ball, chase directly
    if (distToBall < 0.5f) {
        std::cerr << "  Close to ball - chasing directly" << std::endl;
        bot.chaseBall(ball_);
        return;
    }

    // Take advanced pressing position
    float pressX = std::max(ballX, 0.0f) + 0.2f; // Always forward pressure
    float pressZ = ball_.getPosZ() * 0.7f;

    pressX = std::clamp(pressX, 0.0f, FIELD_HALF_LENGTH - 0.3f);
    pressZ = std::clamp(pressZ, -FIELD_HALF_WIDTH + 0.3f, FIELD_HALF_WIDTH - 0.3f);

    std::cerr << "  Pressing position: (" << pressX << ", " << pressZ << ")" << std::endl;
    bot.moveToWhileFacing(pressX, pressZ, ball_.getPosX(), ball_.getPosZ());
}

/**
 * CAMPER: Midfield presence for stuck ball situations
 * - Maintains midfield position waiting for stuck ball
 * - Engages when ball is very close (<0.35m)
 * - Otherwise waits in strategic midfield location
 */
void GameState::executeCamper(Robot& bot, const Robot& teammate) {
    std::cerr << ">> CAMPER" << std::endl;

    // Calculate midfield camping position
    float campX = ball_.getPosX() * 0.7f;
    float campZ = ball_.getPosZ() * 0.5f;

    float distToBall = bot.distanceTo(ball_);

    // If ball comes very close, engage and dispute possession
    if (distToBall < 0.35f) {
        std::cerr << "  Ball nearby - engaging" << std::endl;
        bot.chaseBall(ball_);
        return;
    }

    std::cerr << "  Maintaining camp position" << std::endl;

    // Clamp to midfield area
    campX = std::clamp(campX, -0.6f, 0.6f);
    campZ = std::clamp(campZ, -FIELD_HALF_WIDTH + 0.3f, FIELD_HALF_WIDTH - 0.3f);

    bot.moveToWhileFacing(campX, campZ, ball_.getPosX(), ball_.getPosZ());
}

/**
 * OFFIELD: Robot not in active play
 * - Maintains static position
 * - No movement or actions taken
 */
void GameState::executeOffField(Robot& bot) {
    std::cerr << ">> OFFIELD - Maintaining position" << std::endl;
    bot.holdPosition();
}(campX, campZ, ball_.getPosX(), ball_.getPosZ());
}

void GameState::executeOffField(Robot& bot) {
    std::cerr << ">> OFFIELD - Maintaining position" << std::endl;
    bot.holdPosition();
}

void GameState::update() {
    playerList_[HOMEBOT_1].resetControls();
    playerList_[HOMEBOT_2].resetControls();

    fieldMap_.update(ball_, playerList_[RIVALBOT_1], playerList_[RIVALBOT_2]);

    // 1. Select strategy based on game state
    Strategies selectedStrategy = selectStrategy();
    currentRoles_.currentStrategy = selectedStrategy;

    // 2. Assign roles based on selected strategy
    assignRoles(selectedStrategy);

    std::cerr << "\n[STRATEGY: ";
    switch(selectedStrategy) {
        case Strategies::DEFENSIVE: std::cerr << "DEFENSIVE"; break;
        case Strategies::BALANCED: std::cerr << "BALANCED"; break;
        case Strategies::OFFENSIVE: std::cerr << "OFFENSIVE"; break;
        case Strategies::DEFENSIVE_CAMPING: std::cerr << "DEF_CAMPING"; break;
        case Strategies::OFFENSIVE_CAMPING: std::cerr << "OFF_CAMPING"; break;
        default: std::cerr << "OFFIELD"; break;
    }
    std::cerr << "]" << std::endl;

    // Execute bot1 behavior
    switch(currentRoles_.bot1Role) {
        case Roles::DEFENDER:
            executeDefender(playerList_[HOMEBOT_1], playerList_[HOMEBOT_2]);
            break;
        case Roles::INTERCEPTOR:
            executeInterceptor(playerList_[HOMEBOT_1], playerList_[HOMEBOT_2], true);
            break;
        case Roles::CAMPER:
            executeCamper(playerList_[HOMEBOT_1], playerList_[HOMEBOT_2]);
            break;
        case Roles::SUPPORT:
            executeSupport(playerList_[HOMEBOT_1], playerList_[HOMEBOT_2]);
            break;
        case Roles::ATTACKER:
            executeAttacker(playerList_[HOMEBOT_1], playerList_[HOMEBOT_2]);
            break;
        case Roles::STRIKER:
            executeStriker(playerList_[HOMEBOT_1], playerList_[HOMEBOT_2]);
            break;
        case Roles::OFFIELD:
            executeOffField(playerList_[HOMEBOT_1]);
            break;
        default:
            playerList_[HOMEBOT_1].holdPosition();
    }

    // Execute bot2 behavior
    switch(currentRoles_.bot2Role) {
        case Roles::DEFENDER:
            executeDefender(playerList_[HOMEBOT_2], playerList_[HOMEBOT_1]);
            break;
        case Roles::INTERCEPTOR:
            executeInterceptor(playerList_[HOMEBOT_2], playerList_[HOMEBOT_1], false);
            break;
        case Roles::CAMPER:
            executeCamper(playerList_[HOMEBOT_2], playerList_[HOMEBOT_1]);
            break;
        case Roles::SUPPORT:
            executeSupport(playerList_[HOMEBOT_2], playerList_[HOMEBOT_1]);
            break;
        case Roles::ATTACKER:
            executeAttacker(playerList_[HOMEBOT_2], playerList_[HOMEBOT_1]);
            break;
        case Roles::STRIKER:
            executeStriker(playerList_[HOMEBOT_2], playerList_[HOMEBOT_1]);
            break;
        case Roles::OFFIELD:
            executeOffField(playerList_[HOMEBOT_2]);
            break;
        default:
            playerList_[HOMEBOT_2].holdPosition();
    }
}
