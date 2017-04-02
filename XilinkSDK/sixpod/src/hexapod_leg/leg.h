/*
 * leg.h
 *
 *  Created on: Feb 6, 2017
 *      Author: Phanomphon Yotchon
 */

#ifndef SRC_HEXAPOD_LEG_LEG_H_
#define SRC_HEXAPOD_LEG_LEG_H_

#include "math.h"
#include "sleep.h"
#include "hexapod_config.h"

#include "../hexapod_leg/joint_controls.h"

typedef struct foot_tip_position {
	u8 inverseX, inverseY, inverseZ;
	float x;
	float y;
	float z;
} FootTipPosition;

typedef struct leg {
	u8 legID;
	u8 jointA, jointB, jointC;
	u8 inverseA, inverseB, inverseC;
	FootTipPosition footTip;
	FootTipPosition footTipTarget;
} Leg;

typedef struct jointVar3 {
	float jointA, jointB, jointC;
} Pos3DOF, Speed3DOF;

void LegInitial(Leg *leg, int Id, FootTipPosition ft);

void LegSetPositionDeg(Leg *leg, Pos3DOF targetPos);
Pos3DOF LegGetPresentPositionDeg(Leg *leg);

void LegSetSpeedDeg(Leg *leg, Speed3DOF speed);

void LegMoveToPositionWithDuration(Leg *leg, Pos3DOF targetPos, float sec);

void LegTaskMovement(Leg *leg, float qcycle[][3], int length, int offset, float dt, int round, int inv);

Pos3DOF LegIk(FootTipPosition ft, float z_off);
Pos3DOF LegIk(float x, float y, float z, float z_off);

#endif /* SRC_HEXAPOD_LEG_LEG_H_ */
