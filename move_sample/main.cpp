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

#include <iostream>
#include <string>
#include <exception>
#include <nlohmann/json.hpp>

#include "constants.h"
#include "robot.h"
#include "strategy.h"

using json = nlohmann::json;
using namespace std;

// ============================================================================
// COMMUNICATION FUNCTIONS
// ============================================================================

// Create JSON message for robot commands
json createSetMessage(const RobotCommand& bot1Cmd, const RobotCommand& bot2Cmd) {
    json message = {
        {"type", "set"},
        {"data", {
            {"homeBot1", {
                {"positionXZ", {bot1Cmd.targetX, bot1Cmd.targetZ}},
                {"rotationY", bot1Cmd.targetRotY},
                {"dribbler", bot1Cmd.dribbler},
                {"kick", bot1Cmd.kick},
                {"chirp", bot1Cmd.chip}
            }},
            {"homeBot2", {
                {"positionXZ", {bot2Cmd.targetX, bot2Cmd.targetZ}},
                {"rotationY", bot2Cmd.targetRotY},
                {"dribbler", bot2Cmd.dribbler},
                {"kick", bot2Cmd.kick},
                {"chirp", bot2Cmd.chip}
            }}
        }}
    };
    return message;
}

// Send commands to simulator
void sendCommands(const RobotCommand& bot1Cmd, const RobotCommand& bot2Cmd) {
    json message = createSetMessage(bot1Cmd, bot2Cmd);
    cout << message.dump() << endl;
    cout.flush();
}

// Parse game state from JSON message
GameState parseStateMessage(const json& message) {
    GameState state;

    if (!message.contains("data")) {
        cerr << "Warning: Missing data field" << endl;
        return state;
    }

    const json& data = message["data"];

    // Helper function to read robot state
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
                rs.rotY = obj["rotation"][1] + M_PI;
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
    if (data.contains("homeBot1"))  readRobot(data["homeBot1"], state.homeBot1);
    if (data.contains("homeBot2"))  readRobot(data["homeBot2"], state.homeBot2);
    if (data.contains("rivalBot1")) readRobot(data["rivalBot1"], state.rivalBot1);
    if (data.contains("rivalBot2")) readRobot(data["rivalBot2"], state.rivalBot2);
    if (data.contains("ball"))      readRobot(data["ball"], state.ball);

std:cerr << std::endl << "-------------------------------" << std::endl
              << "Parsed GameState: " << std::endl
              << "  HomeBot1pos(" << state.homeBot1.posX << ", " << state.homeBot1.posY << ", " << state.homeBot1.posZ << "), " << std::endl
              << "  HomeBot1rot(" << state.homeBot1.rotX << ", " << state.homeBot1.rotY << ", " << state.homeBot1.rotZ << "), " << std::endl
              << "  HomeBot2pos(" << state.homeBot2.posX << ", " << state.homeBot2.posY << ", " << state.homeBot2.posZ << "), " << std::endl
              << "  HomeBot2rot(" << state.homeBot2.rotX << ", " << state.homeBot2.rotY << ", " << state.homeBot2.rotZ << "), " << std::endl
              << "  Ball(" << state.ball.posX << ", " << state.ball.posZ << ")" << std::endl
              << std::endl;

    return state;
}

// ============================================================================
// MAIN LOOP
// ============================================================================

int main(int argc, char* argv[]) {
    bool isRunning = false;
    uint32_t frameCount = 0;

    cerr << "=====================================" << endl;
    cerr << "   EDACup 2025 Robot Controller" << endl;
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
                frameCount = 0;
                cerr << ">>> GAME STARTED <<<" << endl;
            }
            else if (type == "stop") {
                isRunning = false;
                cerr << ">>> GAME STOPPED <<<" << endl;
                cerr << "Frames processed: " << frameCount << endl;
            }
            else if (type == "state" && isRunning) {
                frameCount++;

                // Process game state and generate commands
                GameState state = parseStateMessage(message);
                RobotCommand bot1Cmd, bot2Cmd;
                decideStrategy(state, bot1Cmd, bot2Cmd);
                sendCommands(bot1Cmd, bot2Cmd);

                // Status update every 100 frames
                if (frameCount % 100 == 0) {
                    cerr << "[Frame " << frameCount << "] Running" << endl;
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