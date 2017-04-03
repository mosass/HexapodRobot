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
	void move();
public:
	uint8_t id;
	float zOffset;
	bool invX;
	bool invY;
	bool invZ;
	bool invA;
	bool invB;
	bool invC;

	Leg(int Id);
	Link3d getPresentPosition();
	void moveTo(Link3d& targetJointPos, float in_sec);
	void moveTo(FootTip& targetFootTipPos, float in_sec);
};

#endif /* SRC_HEXAPOD_LEG_LEG_H_ */
