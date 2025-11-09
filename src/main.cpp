/**
 * @file main.cpp
 * @brief Main program entry point - Refactored
 * @author Agustin Valenzuela, Alex Petersen, Dylan Frigerio, Enzo Fernandez Rosas
 * @copyright Copyright (c) 2025
 */

#include <exception>
#include <iostream>
#include <string>
#include <nlohmann/json.hpp>
#include "GameState.h"

using json = nlohmann::json;
using namespace std;

// ============================================================================
// GLOBAL GAME STATE
// ============================================================================

GameState g_gameState;

// ============================================================================
// MAIN LOOP
// ============================================================================

int main(int argc, char* argv[]) {
    bool isRunning = false;

    cerr << "=====================================" << endl;
    cerr << "   EDACup 2025 Robot Controller" << endl;
    cerr << "=====================================" << endl;
    cerr << "Field layout:" << endl;
    cerr << "  X axis: LENGTH (±" << FIELD_HALF_LENGTH << "m)" << endl;
    cerr << "  Z axis: WIDTH (±" << FIELD_HALF_WIDTH << "m)" << endl;
    cerr << "  Own goal: X = " << LEFT_GOAL_X << endl;
    cerr << "  Opponent goal: X = " << RIGHT_GOAL_X << endl;
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
                // Game started
                isRunning = true;
                g_gameState.reset();
                cerr << "\n>>> GAME STARTED <<<\n" << endl;

            }
            else if (type == "stop") {
                // Game stopped
                isRunning = false;
                cerr << "\n>>> GAME STOPPED <<<" << endl;
                cerr << "Total frames: " << g_gameState.getFrameCount() << endl;

            }
            else if (type == "state" && isRunning) {
                // New frame received
                g_gameState.nextFrame();

                // Parse state from JSON
                g_gameState.parseFromJson(message);

                // Execute strategy (updates robot commands)
                g_gameState.update();

                // Send commands to simulator
                json commandMsg = g_gameState.createCommandJson();
                cout << commandMsg.dump() << endl;
                cout.flush();

                // Periodic status update
                if (g_gameState.getFrameCount() % 200 == 0) {
                    cerr << "[Frame " << g_gameState.getFrameCount() << "] Active" << endl;
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