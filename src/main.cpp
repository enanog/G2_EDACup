/**
 * @file main.cpp
 * @brief Main program entry point - Threaded version
 * @author Agustin Valenzuela, Alex Petersen, Dylan Frigerio, Enzo Fernandez Rosas
 * @copyright Copyright (c) 2025
 */

#include <atomic>
#include <chrono>
#include <exception>
#include <iostream>
#include <mutex>
#include <nlohmann/json.hpp>
#include <string>
#include <thread>

#include "gamestate.h"

using json = nlohmann::json;
using namespace std;

// ============================================================================
// GLOBAL GAME STATE & SYNCHRONIZATION
// ============================================================================

GameState g_gameState;
mutex g_stateMutex;                // Protects access to g_gameState
atomic<bool> g_isRunning(false);   // Game active flag
atomic<bool> g_shouldExit(false);  // Program exit flag

// Command buffer for thread-safe communication
json g_latestCommand;
mutex g_commandMutex;
bool g_hasNewCommand = false;

// ============================================================================
// GAME LOGIC THREAD
// ============================================================================

void gameLogicThread() {
    const int UPDATE_RATE_MS = 50;  // Match simulator rate

    cerr << "[LOGIC] Game logic thread started" << endl;

    while (!g_shouldExit) {
        auto frameStart = chrono::steady_clock::now();

        // Only update if game is running
        if (g_isRunning) {
            {
                lock_guard<mutex> lock(g_stateMutex);

                // Increment frame counter
                g_gameState.nextFrame();

                // Execute strategy (updates robot commands)
                g_gameState.update();

                // Generate command JSON
                json commandMsg = g_gameState.createCommandJson();

                // Store in command buffer
                {
                    lock_guard<mutex> cmdLock(g_commandMutex);
                    g_latestCommand = commandMsg;
                    g_hasNewCommand = true;
                }

                // Periodic status update
                if (g_gameState.getFrameCount() % 200 == 0) {
                    cerr << "[LOGIC] Frame " << g_gameState.getFrameCount() << " - Active" << endl;
                }
            }
        }

        // Sleep to maintain consistent update rate
        auto frameEnd = chrono::steady_clock::now();
        auto elapsed = chrono::duration_cast<chrono::milliseconds>(frameEnd - frameStart);
        auto sleepTime = chrono::milliseconds(UPDATE_RATE_MS) - elapsed;

        if (sleepTime.count() > 0) {
            this_thread::sleep_for(sleepTime);
        } else {
            cerr << "[LOGIC] Warning: Update took " << elapsed.count() << "ms (exceeds "
                 << UPDATE_RATE_MS << "ms budget)" << endl;
        }
    }

    cerr << "[LOGIC] Game logic thread exiting" << endl;
}

// ============================================================================
// COMMUNICATION THREAD
// ============================================================================

void communicationThread() {
    cerr << "=====================================" << endl;
    cerr << "   EDACup 2025 Robot Controller" << endl;
    cerr << "=====================================" << endl;
    cerr << "Field layout:" << endl;
    cerr << "  X axis: LENGTH (±" << FIELD_HALF_LENGTH << "m)" << endl;
    cerr << "  Z axis: WIDTH (±" << FIELD_HALF_WIDTH << "m)" << endl;
    cerr << "  Own goal: X = " << LEFT_GOAL_X << endl;
    cerr << "  Opponent goal: X = " << RIGHT_GOAL_X << endl;
    cerr << "=====================================" << endl;
    cerr << "[COMM] Waiting for game messages..." << endl;

    while (!g_shouldExit) {
        try {
            string line;
            if (!getline(cin, line)) {
                cerr << "[COMM] Input closed, signaling exit." << endl;
                g_shouldExit = true;
                break;
            }

            if (line.empty())
                continue;

            // Parse JSON message
            json message = json::parse(line);
            string type = message.value("type", "");

            if (type == "start") {
                // Game started
                cerr << "\n[COMM] >>> GAME STARTED <<<\n" << endl;

                {
                    lock_guard<mutex> lock(g_stateMutex);
                    g_gameState.reset();
                }

                g_isRunning = true;

            } else if (type == "stop") {
                // Game stopped
                g_isRunning = false;

                cerr << "\n[COMM] >>> GAME STOPPED <<<" << endl;

                {
                    lock_guard<mutex> lock(g_stateMutex);
                    cerr << "[COMM] Total frames: " << g_gameState.getFrameCount() << endl;
                }

            } else if (type == "state") {
                // New state received - update game state
                {
                    lock_guard<mutex> lock(g_stateMutex);
                    g_gameState.parseFromJson(message);
                }

                // Send any pending commands
                {
                    lock_guard<mutex> cmdLock(g_commandMutex);
                    if (g_hasNewCommand) {
                        cout << g_latestCommand.dump() << endl;
                        cout.flush();
                        g_hasNewCommand = false;
                    }
                }
            }

        } catch (const exception& e) {
            cerr << "[COMM] Error: " << e.what() << endl;
        }
    }

    cerr << "[COMM] Communication thread exiting" << endl;
}

// ============================================================================
// MAIN ENTRY POINT
// ============================================================================

void test() {
    cerr << "Test thread started" << endl;
    const int UPDATE_RATE_MS = 50;  // Match simulator rate
    int8_t ready = 0;

    while (!g_shouldExit) {
        auto frameStart = chrono::steady_clock::now();

        // Only update if game is running
        if (g_isRunning) {
            {
                lock_guard<mutex> lock(g_stateMutex);

                // Increment frame counter
                g_gameState.nextFrame();

                g_gameState.getBot(HOMEBOT_1).holdPosition();
                if(ready != 2){
                    if (!g_gameState.getBot(HOMEBOT_2).hasBallControl(g_gameState.getBall()) &&
                        ready == 0) {
                        g_gameState.getBot(HOMEBOT_2).faceTowards(g_gameState.getBall().getPosX(),
                                                                  g_gameState.getBall().getPosZ());
                        g_gameState.getBot(HOMEBOT_2).dribbleTo(0.0f, 0.0f);
                    } else {
                        ready = 1;
                    }
                }

                if (ready == 1) {
                    if (g_gameState.quickPass() == 1)
                        ready = 2;
                }

                g_gameState.getBot(HOMEBOT_1).setDribbler(1.0f);

                // Generate command JSON
                json commandMsg = g_gameState.createCommandJson();

                // Store in command buffer
                {
                    lock_guard<mutex> cmdLock(g_commandMutex);
                    g_latestCommand = commandMsg;
                    g_hasNewCommand = true;
                }

                // Periodic status update
                if (g_gameState.getFrameCount() % 200 == 0) {
                    cerr << "[LOGIC] Frame " << g_gameState.getFrameCount() << " - Active" << endl;
                }
            }
        }

        // Sleep to maintain consistent update rate
        auto frameEnd = chrono::steady_clock::now();
        auto elapsed = chrono::duration_cast<chrono::milliseconds>(frameEnd - frameStart);
        auto sleepTime = chrono::milliseconds(UPDATE_RATE_MS) - elapsed;

        if (sleepTime.count() > 0) {
            this_thread::sleep_for(sleepTime);
        } else {
            cerr << "[LOGIC] Warning: Update took " << elapsed.count() << "ms (exceeds "
                 << UPDATE_RATE_MS << "ms budget)" << endl;
        }
    }
}

int main() {
    thread commThread(communicationThread);
    thread testThread(test);
    testThread.join();
    commThread.join();
    // try {
    //     // Start both threads
    //     thread logicThread(gameLogicThread);
    //     thread commThread(communicationThread);

    //     // Wait for both threads to complete
    //     commThread.join();
    //     logicThread.join();

    //     cerr << "[MAIN] Controller shutdown complete." << endl;

    // } catch (const exception& e) {
    //     cerr << "[MAIN] Fatal error: " << e.what() << endl;
    //     return 1;
    // }

    return 0;
}
