/**
 * @file main.cpp
 * @brief Main program entry point
 * @author Agustin Valenzuela,
 *         Alex Petersen,
 *         Dylan Frigerio,
 *         Enzo Fernandez Rosas
 *
 * @copyright Copyright (c) 2025
 */

#include <cmath>
#include <exception>
#include <iostream>
#include <nlohmann/json.hpp>
#include <string>

#include "constants.h"
#include "robot.h"
#include "strategy.h"

using json = nlohmann::json;
using namespace std;

// ============================================================================
// COMMUNICATION FUNCTIONS
// ============================================================================

/**
 * Create JSON message for robot commands
 */
json createSetMessage(const RobotCommand& bot1Cmd, const RobotCommand& bot2Cmd) {
    json message = {
        {"type", "set"},
        {"data", {
            {"homeBot1", {
                {"positionXZ", {bot1Cmd.targetX, bot1Cmd.targetZ}},
                {"rotationY", bot1Cmd.targetRotY},
                {"dribbler", bot1Cmd.dribbler},
                {"kick", bot1Cmd.kick},
                {"chip", bot1Cmd.chip}
            }},
            {"homeBot2", {
                {"positionXZ", {bot2Cmd.targetX, bot2Cmd.targetZ}},
                {"rotationY", bot2Cmd.targetRotY},
                {"dribbler", bot2Cmd.dribbler},
                {"kick", bot2Cmd.kick},
                {"chip", bot2Cmd.chip}
            }}
        }}
    };
    return message;
}

/**
 * Send commands to simulator
 */
void sendCommands(const RobotCommand& bot1Cmd, const RobotCommand& bot2Cmd) {
    json message = createSetMessage(bot1Cmd, bot2Cmd);
    cout << message.dump() << endl;
    cout.flush();
}

/**
 * Parse game state from JSON message
 */
GameState parseStateMessage(const json& message) {
    GameState state;

    if (!message.contains("data")) {
        cerr << "Warning: Missing data field" << endl;
        return state;
    }

    const json& data = message["data"];

    // Helper lambda to read robot/ball state
    auto readRobot = [&](const json& obj, RobotState& rs) {
        try {
            if (obj.contains("position") && obj["position"].is_array() &&
                obj["position"].size() >= 3) {
                rs.posX = obj["position"][0];
                rs.posY = obj["position"][1];
                rs.posZ = obj["position"][2];
            }

            if (obj.contains("rotation") && obj["rotation"].is_array() &&
                obj["rotation"].size() >= 3) {
                rs.rotX = obj["rotation"][0];
                rs.rotY = obj["rotation"][1];
                rs.rotZ = obj["rotation"][2];
            }

            if (obj.contains("velocity") && obj["velocity"].is_array() &&
                obj["velocity"].size() >= 3) {
                rs.velX = obj["velocity"][0];
                rs.velY = obj["velocity"][1];
                rs.velZ = obj["velocity"][2];
            }
        }
        catch (const exception& e) {
            cerr << "Error reading robot state: " << e.what() << endl;
        }
        };

    // Read all robots and ball
    if (data.contains("homeBot1")) readRobot(data["homeBot1"], state.homeBot1);
    if (data.contains("homeBot2")) readRobot(data["homeBot2"], state.homeBot2);
    if (data.contains("rivalBot1")) readRobot(data["rivalBot1"], state.rivalBot1);
    if (data.contains("rivalBot2")) readRobot(data["rivalBot2"], state.rivalBot2);
    if (data.contains("ball")) readRobot(data["ball"], state.ball);

    return state;
}

// ============================================================================
// MAIN LOOP
// ============================================================================

int main(int argc, char* argv[]) {
    bool isRunning = false;
    GameState state;

    cerr << "=====================================" << endl;
    cerr << "   EDACup 2025 Robot Controller" << endl;
    cerr << "=====================================" << endl;
    cerr << "Field layout:" << endl;
    cerr << "  X axis: LENGTH (±1.095m)" << endl;
    cerr << "  Z axis: WIDTH (±0.79m)" << endl;
    cerr << "  Own goal: X = -1.095" << endl;
    cerr << "  Opponent goal: X = +1.095" << endl;
    cerr << "=====================================" << endl;
    cerr << "Waiting for game messages..." << endl;

    while (true) {
        try {
            string line;
            if (!getline(cin, line)) {
                cerr << "Input closed, exiting." << endl;
                break;
            }

            if (line.empty()) continue;

            // Parse JSON message
            json message = json::parse(line);
            string type = message.value("type", "");

            if (type == "start") {
                isRunning = true;
                state.frameCount = 0;
                cerr << "\n>>> GAME STARTED <<<\n" << endl;

            }
            else if (type == "stop") {
                isRunning = false;
                cerr << "\n>>> GAME STOPPED <<<" << endl;
                cerr << "Total frames: " << state.frameCount << endl;

            }
            else if (type == "state" && isRunning) {
                state.frameCount++;

                // Parse incoming state
                GameState newState = parseStateMessage(message);

                // Copy parsed state (keep frameCount from previous state)
                uint32_t currentFrame = state.frameCount;
                state = newState;
                state.frameCount = currentFrame;

                // Execute strategy (modifies state.bot1Cmd and state.bot2Cmd)
                decideStrategy(state);

                // Send commands to simulator
                sendCommands(state.bot1Cmd, state.bot2Cmd);

                // Periodic status update (less verbose)
                if (state.frameCount % 200 == 0) {
                    cerr << "[Frame " << state.frameCount << "] Active" << endl;
                }
            }

        }
        catch (const exception& e) {
            cerr << "Error: " << e.what() << endl;
        }
    }

    cerr << "Controller shutdown complete." << endl;
    return 0;
}