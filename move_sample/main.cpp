/*
 * @brief Main program loop and JSON communication with simulator
 * @author Agustin Valenzuela,
 *         Alex Petersen,
 *         Dylan Frigerio,
 *         Enzo Fernandez Rosas
 *
 * @copyright Copyright(c) 2025
 */

#include <iostream>
#include <string>
#include <exception>
#include <nlohmann/json.hpp>

#include "constants.h"
#include "robot.h"
#include "strategy.h"
#include "geometry.h"

using json = nlohmann::json;
using namespace std;

// ============================================================================
// COMMUNICATION FUNCTIONS
// ============================================================================

/**
 * @brief Create JSON message to send robot commands
 * @param bot1Cmd Command for robot 1
 * @param bot2Cmd Command for robot 2
 * @return JSON message in correct format
 */
json createSetMessage(const RobotCommand& bot1Cmd, const RobotCommand& bot2Cmd);

/**
 * @brief Parse incoming state message from simulator
 * @param message JSON message received
 * @return Parsed game state
 */
GameState parseStateMessage(const json& message);

/**
 * @brief Send commands to simulator via stdout
 * @param bot1Cmd Command for robot 1
 * @param bot2Cmd Command for robot 2
 */
void sendCommands(const RobotCommand& bot1Cmd, const RobotCommand& bot2Cmd);


int main(int argc, char* argv[])
{
	bool isRunning = false;
	uint32_t time = 0;
	// Debug prints to stderr so simulator stdout remains clean for JSON outputs.
	cerr << "EDACup minimal controller started" << endl;
	cerr << "Waiting for messages..." << endl;

	while (true)
	{
		try
		{
			string line;
			if (!std::getline(cin, line)) {
				// EOF or pipe closed: exit cleanly
				cerr << "stdin closed, exiting." << endl;
				break;
			}

			if (line.empty()) continue; // skip empty lines

			// parse incoming json
			json message = json::parse(line);
			string type = message.value("type", "");
			if (type == "start") {
				isRunning = true;
				cerr << "Received start" << endl;
			}
			else if (type == "stop") {
				isRunning = false;
				cerr << "Received stop" << endl;
			}
			else if (type == "state") {
				// Only act when running.
				if (!isRunning) continue;

				// Convert JSON into our minimal GameState
				GameState state = parseStateMessage(message);
				RobotCommand bot1Cmd;

				if (time == 40)
				{
					RobotCommand bot1Cmd = moveToPosition(state.homeBot1, -0.79f, 1.95f);
					cerr << "esquina arriba derecha" << endl;
				}
				else if (80 == time)
				{
					RobotCommand bot1Cmd = moveToPosition(state.homeBot2, 0.79f, 1.95f);
					cerr << "esquina abajo derecha" << endl;
				}
				else if (120 == time)
				{
					RobotCommand bot1Cmd = moveToPosition(state.homeBot1, 0.79f, -1.95f);
					cerr << "esquina abajo izquierda" << endl;
				}
				else if (160 == time)
				{
					RobotCommand bot1Cmd = moveToPosition(state.homeBot1, -0.79f, -1.95f);
					cerr << "esquina arriba izquierda" << endl;
				}
				time++;
				if(time == 160)
					time = 0;

				RobotCommand bot2Cmd = moveToPosition(state.homeBot2, state.ball.posX, state.ball.posZ);
				//decideStrategy(state, bot1Cmd, bot2Cmd);

				// Send commands back to simulator
				sendCommands(bot1Cmd, bot2Cmd);
				
			}
		}
		catch (const std::exception& e) {
			// Print errors to stderr (simulator ignores stderr).
			cerr << "Error in main loop: " << e.what() << endl;
		}
	}

	return 0;
}

json createSetMessage(const RobotCommand& bot1Cmd, const RobotCommand& bot2Cmd)
{
	json message =
	{
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

GameState parseStateMessage(const json& message)
{
	GameState state; 

	// get data object, bail out early if missing
	if (!message.contains("data")) return state;
	const json& data = message["data"];

	auto readRobot = [&](const json& obj, RobotState& rs) {
		// position: [x,y,z] if present
		if (obj.contains("position") && obj["position"].is_array() && obj["position"].size() >= 3) {
			rs.posX = obj["position"][0].get<float>();
			rs.posY = obj["position"][1].get<float>();
			rs.posZ = obj["position"][2].get<float>();
		}

		// rotation: [x,y,z]
		if (obj.contains("rotation") && obj["rotation"].is_array() && obj["rotation"].size() >= 3) {
			rs.rotX = obj["rotation"][0].get<float>();
			rs.rotY = obj["rotation"][1].get<float>();
			rs.rotZ = obj["rotation"][2].get<float>();
		}

		// velocity: [x,y,z]
		if (obj.contains("velocity") && obj["velocity"].is_array() && obj["velocity"].size() >= 3) {
			rs.velX = obj["velocity"][0].get<float>();
			rs.velY = obj["velocity"][1].get<float>();
			rs.velZ = obj["velocity"][2].get<float>();
		}

		// angularVelocity: [x,y,z]
		if (obj.contains("angularVelocity") && obj["angularVelocity"].is_array() && obj["angularVelocity"].size() >= 3) {
			rs.angVelX = obj["angularVelocity"][0].get<float>();
			rs.angVelY = obj["angularVelocity"][1].get<float>();
			rs.angVelZ = obj["angularVelocity"][2].get<float>();
		}
		};

	// For each expected robot/ball key, check existence before parsing.
	if (data.contains("homeBot1"))	readRobot(data["homeBot1"], state.homeBot1);
	if (data.contains("homeBot2"))	readRobot(data["homeBot2"], state.homeBot2);
	if (data.contains("rivalBot1")) readRobot(data["rivalBot1"], state.rivalBot1);
	if (data.contains("rivalBot2")) readRobot(data["rivalBot2"], state.rivalBot2);
	if (data.contains("ball"))      readRobot(data["ball"], state.ball);

	cerr << "Parsed state: "
		 << "HomeBot1(" << state.homeBot1.posX << "," << state.homeBot1.posZ << ") "
		 /* << "HomeBot2(" << state.homeBot2.posX << "," << state.homeBot2.posZ << ") "
		 << "RivalBot1(" << state.rivalBot1.posX << "," << state.rivalBot1.posZ << ") "
		 << "RivalBot2(" << state.rivalBot2.posX << "," << state.rivalBot2.posZ << ") "
		 << "Ball(" << state.ball.posX << "," << state.ball.posZ << ")"*/
		<< endl;

	return state;
}

void sendCommands(const RobotCommand& bot1Cmd, const RobotCommand& bot2Cmd)
{
	json message = createSetMessage(bot1Cmd, bot2Cmd);

	// Use cout for simulator communication (one line).
	cout << message.dump() << endl;
	cout.flush();
}