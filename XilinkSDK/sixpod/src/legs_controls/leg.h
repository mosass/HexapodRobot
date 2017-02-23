/*
 * leg.h
 *
 *  Created on: Feb 6, 2017
 *      Author: Phanomphon Yotchon
 */

#include "joint_controls.h"
#include "sleep.h"

#ifndef SRC_LEGS_CONTROLS_LEG_H_
#define SRC_LEGS_CONTROLS_LEG_H_

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
} Leg;

typedef struct vector3dof {
	float jointA, jointB, jointC;
} Pos3DOF, Speed3DOF;

void LegInitial(Leg *leg, int Id);

void LegSetPositionDeg(Leg *leg, Pos3DOF targetPos);
Pos3DOF LegGetPresentPositionDeg(Leg *leg);

void LegSetSpeedDeg(Leg *leg, Speed3DOF speed);

void LegMoveToPositionWithDuration(Leg *leg, Pos3DOF targetPos, float sec);

void LegTaskMovement(Leg *leg, float qcycle[][3], int length, int offset, float dt, int round, int inv);

float _diff(float A, float B);

#endif /* SRC_LEGS_CONTROLS_LEG_H_ */
