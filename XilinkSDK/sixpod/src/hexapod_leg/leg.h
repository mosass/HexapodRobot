/*
 * leg.h
 *
 *  Created on: Feb 6, 2017
 *      Author: Phanomphon Yotchon
 */

#include "leg_type.h"

#ifndef SRC_HEXAPOD_LEG_LEG_H_
#define SRC_HEXAPOD_LEG_LEG_H_

class Leg {
private:
	uint8_t jointIdA;
	uint8_t jointIdB;
	uint8_t jointIdC;

	FootTip footTipPos;
	Link3d	linkPos;
	Link3d	linkSpeed;

	Link3d calcIk();
	void move(bool skipSetSpeed);
public:
	uint8_t id;
	float zOffset;
	bool invX;
	bool invY;
	bool invZ;
	bool invA;
	bool invB;
	bool invC;

	void setup(int Id, float z_off);
	void setGoalPosition(FootTip& targetFootTipPos);
	Link3d getPresentPosition();
	void moveTo(FootTip& targetFootTipPos, float in_sec);
};

#endif /* SRC_HEXAPOD_LEG_LEG_H_ */
