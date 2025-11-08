// ============================================================================
// main.cpp
// Main program - handles simulator communication and game loop
// ============================================================================

#include "game_state.h"
#include "geometry.h"
#include "actions.h"
#include "strategy.h"
#include "utils.h"
#include <nlohmann/json.hpp>
#include <iostream>
#include <string>

using json = nlohmann::json;
using namespace std;

/**
 * Parse robot data from JSON message
 */
void parseRobot(const json& data, Robot& robot) {
    // Parse position (3D vector, we use x and z)
    if (data.contains("position") && data["position"].is_array() && data["position"].size() >= 3) {
        robot.pos.x = data["position"][0];
        robot.pos.z = data["position"][2];
    }

    // Parse rotation (Euler angles, we use Y rotation)
    if (data.contains("rotation") && data["rotation"].is_array() && data["rotation"].size() >= 3) {
        robot.rotY = data["rotation"][1];
    }

    // Parse velocity (3D vector, we use x and z)
    if (data.contains("velocity") && data["velocity"].is_array() && data["velocity"].size() >= 3) {
        robot.vel.x = data["velocity"][0];
        robot.vel.z = data["velocity"][2];
    }
}

/**
 * Parse ball data from JSON message
 */
void parseBall(const json& data, Ball& ball) {
    // Parse position
    if (data.contains("position") && data["position"].is_array() && data["position"].size() >= 3) {
        ball.pos.x = data["position"][0];
        ball.pos.z = data["position"][2];
    }

    // Parse velocity
    if (data.contains("velocity") && data["velocity"].is_array() && data["velocity"].size() >= 3) {
        ball.vel.x = data["velocity"][0];
        ball.vel.z = data["velocity"][2];
    }
}

/**
 * Parse complete game state from JSON message
 */
bool parseGameState(const json& message, GameState& state) {
    if (!message.contains("data")) {
        return false;
    }

    const json& data = message["data"];

    // Parse all robots
    if (data.contains("homeBot1")) parseRobot(data["homeBot1"], state.homeBot1);
    if (data.contains("homeBot2")) parseRobot(data["homeBot2"], state.homeBot2);
    if (data.contains("rivalBot1")) parseRobot(data["rivalBot1"], state.rivalBot1);
    if (data.contains("rivalBot2")) parseRobot(data["rivalBot2"], state.rivalBot2);

    // Parse ball
    if (data.contains("ball")) parseBall(data["ball"], state.ball);

    return true;
}

/**
 * Send robot commands to simulator
 */
void sendCommands(const json& bot1Command, const json& bot2Command) {
    json response = {
        {"type", "set"},
        {"data", {
            {"homeBot1", bot1Command},
            {"homeBot2", bot2Command}
        }}
    };

    // Send to simulator via stdout
    cout << response.dump() << endl;
    cout.flush();
}

/**
 * Main game loop
 */
int main() {
    // Print startup banner
    cerr << "============================================" << endl;
    cerr << "  EDACup Robot Soccer Strategy v1.0" << endl;
    cerr << "  Modular C++ Implementation" << endl;
    cerr << "============================================" << endl;
    cerr << "Coordinate System:" << endl;
    cerr << "  X axis: Field WIDTH (±0.79m - sidelines)" << endl;
    cerr << "  Z axis: Field LENGTH (±1.095m - goal lines)" << endl;
    cerr << "  Own goal: Z = -1.095 (backward)" << endl;
    cerr << "  Opponent goal: Z = +1.095 (forward)" << endl;
    cerr << "  rotY = 0 → faces +X" << endl;
    cerr << "  rotY = π/2 → faces +Z (toward opponent goal)" << endl;
    cerr << "============================================" << endl;
    cerr << "Strategy:" << endl;
    cerr << "  homeBot1 (Attacker): Chase, dribble, shoot" << endl;
    cerr << "  homeBot2 (Defender): Protect goal, clear ball" << endl;
    cerr << "============================================" << endl << endl;

    GameState state;

    // Main communication loop with simulator
    while (true) {
        try {
            // Read line from simulator (stdin)
            string line;
            if (!getline(cin, line)) {
                break; // End of input
            }

            if (line.empty()) {
                continue; // Skip empty lines
            }

            // Parse JSON message
            json message = json::parse(line);
            string messageType = message.value("type", "");

            // Handle different message types
            if (messageType == "start") {
                // Game started
                state.isPlaying = true;
                state.frameCount = 0;
                cerr << "\n>>> GAME STARTED <<<\n" << endl;

            }
            else if (messageType == "stop") {
                // Game stopped
                state.isPlaying = false;
                cerr << "\n>>> GAME STOPPED <<<\n" << endl;

            }
            else if (messageType == "state" && state.isPlaying) {
                // Receive game state and compute response
                state.frameCount++;

                if (parseGameState(message, state)) {
                    // Update rule tracking
                    // updateRobotRules(state.homeBot1, state.ball);
                    // updateRobotRules(state.homeBot2, state.ball);
                    updateBallTracking(state.ball);

                    // Execute strategies for both robots
                    json bot1Command, bot2Command;

                    executeAttacker(state, bot1Command);
                    executeDefender(state, bot2Command);

                    // Send commands back to simulator
                    sendCommands(bot1Command, bot2Command);

                    // Periodic status logging
                    if (state.frameCount % 200 == 0) {
                        cerr << "\n--- Frame " << state.frameCount << " Status ---" << endl;
                        cerr << "Ball: (" << state.ball.pos.x << ", " << state.ball.pos.z
                            << ") vel: " << state.ball.vel.length() << " m/s" << endl;
                        logRobotState("Attacker", state.homeBot1);
                        logRobotState("Defender", state.homeBot2);
                        cerr << endl;
                    }
                }
            }

        }
        catch (const exception& e) {
            // Handle any errors gracefully
            cerr << "ERROR: " << e.what() << endl;
        }
    }

    cerr << "\n>>> Program terminated <<<" << endl;
    return 0;
}