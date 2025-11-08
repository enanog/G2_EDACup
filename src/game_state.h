// ============================================================================
// game_state.h
// Defines data structures for robots, ball, and game state
// ============================================================================

#ifndef GAME_STATE_H
#define GAME_STATE_H

#include "geometry.h"

/**
 * Robot structure - contains position, rotation, velocity, and rule tracking
 */
struct Robot {
    Vec2 pos;              // Position on field (x, z)
    double rotY;           // Rotation around Y axis (radians)
    Vec2 vel;              // Velocity vector (m/s)
    
    // Rule enforcement tracking
    int timeoutFrames;     // Frames remaining in timeout (0 = active)
    double ballTouchTime;  // Continuous time touching ball (seconds)
    bool wasTouchingBall;  // Previous frame ball touch state
    
    Robot() : pos(0, 0), rotY(0), vel(0, 0), 
              timeoutFrames(0), ballTouchTime(0), wasTouchingBall(false) {}
};

/**
 * Ball structure - contains position, velocity, and stationarity tracking
 */
struct Ball {
    Vec2 pos;              // Position on field (x, z)
    Vec2 vel;              // Velocity vector (m/s)
    double stationaryTime; // Time ball has been stationary (seconds)
    Vec2 lastPos;          // Position in previous frame
    
    Ball() : pos(0, 0), vel(0, 0), stationaryTime(0), lastPos(0, 0) {}
};

/**
 * Complete game state - all robots, ball, and game status
 */
struct GameState {
    Robot homeBot1;    // Our attacker
    Robot homeBot2;    // Our defender
    Robot rivalBot1;   // Opponent robot 1
    Robot rivalBot2;   // Opponent robot 2
    Ball ball;         // The ball
    
    int frameCount;    // Total frames since game start
    bool isPlaying;    // Whether game is active
    
    GameState() : frameCount(0), isPlaying(false) {}
};

#endif // GAME_STATE_H
