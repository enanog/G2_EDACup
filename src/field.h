#ifndef FIELD_H
#define FIELD_H

/**
 * @brief Field mapping for tactical awareness
 *
 * Tracks penalty areas, predicted ball position, threats, and tactical info
 */
struct FieldMap {
	// Penalty area boundaries
	float ownPenaltyMinX;
	float ownPenaltyMaxX;
	float oppPenaltyMinX;
	float oppPenaltyMaxX;
	float penaltyMinZ;
	float penaltyMaxZ;

	// Predicted positions (latency compensation)
	float predictedBallX;
	float predictedBallZ;
	float predictedBallVelX;
	float predictedBallVelZ;

	// Tactical information
	float ballDistToOwnGoal;
	float ballDistToOppGoal;
	bool isBallStuck;
	int activeOpponents;

	// Threat assessment
	float closestRivalToOwnGoal;
	const Robot* closestRivalPtr;

	FieldMap();

	/**
	 * @brief Update field map with current game state
	 */
	void update(const Ball& ball, const Robot& rival1, const Robot& rival2);

	/**
	 * @brief Check if position is inside own penalty area
	 */
	bool inOwnPenaltyArea(float x, float z) const;

	/**
	 * @brief Check if position is inside opponent penalty area
	 */
	bool inOppPenaltyArea(float x, float z) const;

	/**
	 * @brief Get safe position outside penalty area
	 */
	void getSafePosition(float& x, float& z) const;

	/**
	 * @brief Check if pass line is clear (no obstacles)
	 */
	bool isPassLineClear(const Robot& passer,
						 const Robot& receiver,
						 const Robot& rival1,
						 const Robot& rival2) const;

	/**
	 * @brief Calculate pass quality score (0.0 = bad, 1.0 = excellent)
	 */
	float calculatePassQuality(const Robot& passer,
							   const Robot& receiver,
							   const Robot& rival1,
							   const Robot& rival2) const;

	/**
	 * @brief Check if shooting path is blocked by rivals
	 */
	bool isShootingPathBlocked(const Robot& shooter,
							   const Robot& rival1,
							   const Robot& rival2) const;
};

#endif   // FIELD_H
