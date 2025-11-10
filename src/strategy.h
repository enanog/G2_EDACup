#ifndef STRATEGY_H
#define STRATEGY_H

enum Roles {
    DEFENDER,     // Goalkeeper
    INTERCEPTOR,  // Follows the ball when not in possession
    CAMPER,       // Midfielder (when ball is stuck)
    SUPPORT,      // Assists
    ATTACKER,     // Mostly on rivals side
    STRIKER,      // Waits for pass to score
    OFFIELD       // Off-field (disabled)
};

enum Strategies {
    DEFENSIVE,          // One defender, one interceptor
    BALANCED,           // One defender, one attacker
    OFFENSIVE_CAMPING,  // One camper, one attacker
    DEFENSIVE_CAMPING,  // One defender, one camper
    OFFENSIVE,          // One support, one striker
    OFFIELD_STRATEGY,   // One defender
    NO_STRATEGY                // No robots on field
};

struct RoleAssignment {
    Roles bot1Role;
    Roles bot2Role;
    Strategies currentStrategy;

    RoleAssignment()
        : bot1Role(Roles::DEFENDER),
          bot2Role(Roles::INTERCEPTOR),
          currentStrategy(Strategies::DEFENSIVE) {}
};

#endif  // STRATEGY_H
