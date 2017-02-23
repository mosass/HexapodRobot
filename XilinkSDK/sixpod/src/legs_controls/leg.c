/*
 * leg.c
 *
 *  Created on: Feb 6, 2017
 *      Author: Phanomphon Yotchon
 */

#include "leg.h"

void LegInitial(Leg *leg, int Id) {
	leg->legID = Id;
	leg->jointA = (Id-1)*3 + 1;
	leg->jointB = (Id-1)*3 + 2;
	leg->jointC = (Id-1)*3 + 3;

	leg->inverseA = FALSE;
	leg->inverseB = FALSE;
	leg->inverseC = FALSE;

	return;
}

void LegSetPositionDeg(Leg *leg, Pos3DOF targetPos) {
	if (leg->inverseA) {
		targetPos.jointA = 300 - targetPos.jointA;
	}

	if (leg->inverseB) {
		targetPos.jointB = 300 - targetPos.jointB;
	}

	if (leg->inverseC) {
		targetPos.jointC = 300 - targetPos.jointC;
	}

	JointSetGoalPositionDeg(leg->jointA, targetPos.jointA);
	JointSetGoalPositionDeg(leg->jointB, targetPos.jointB);
	JointSetGoalPositionDeg(leg->jointC, targetPos.jointC);

	return;
}

Pos3DOF LegGetPresentPositionDeg(Leg *leg) {
	Pos3DOF position;
	position.jointA = JointGetPresentPositionDeg(leg->jointA);
	position.jointB = JointGetPresentPositionDeg(leg->jointB);
	position.jointC = JointGetPresentPositionDeg(leg->jointC);

	if (leg->inverseA) {
		position.jointA = 300 - position.jointA;
	}

	if (leg->inverseB) {
		position.jointB = 300 - position.jointB;
	}

	if (leg->inverseC) {
		position.jointC = 300 - position.jointC;
	}

	return position;
}

void LegSetSpeedDeg(Leg *leg, Speed3DOF speed) {
	JointSetMovingSpeedDeg(leg->jointA, speed.jointA);
	JointSetMovingSpeedDeg(leg->jointB, speed.jointB);
	JointSetMovingSpeedDeg(leg->jointC, speed.jointC);

	return;
}

void LegMoveToPositionWithDuration(Leg *leg, Pos3DOF targetPos, float sec) {
	Pos3DOF pos3d = LegGetPresentPositionDeg(leg);
	Speed3DOF speed3d;

	speed3d.jointA = _diff(targetPos.jointA, pos3d.jointA) / sec;
	speed3d.jointB = _diff(targetPos.jointB, pos3d.jointB) / sec;
	speed3d.jointC = _diff(targetPos.jointC, pos3d.jointC) / sec;

	LegSetSpeedDeg(leg, speed3d);
	LegSetPositionDeg(leg, targetPos);

	return;
}

//void LegTaskMovement(Leg *leg, float qcycle[][3], int length, float dt, int offset) {
//	int k = offset;
//	Pos3DOF targetPos;
//
//	targetPos.jointA = qcycle[k][0];
//	targetPos.jointB = qcycle[k][1];
//	targetPos.jointC = qcycle[k][2];
//	LegMoveToPositionWithDuration(leg, targetPos, 2);
//	sleep(2);
//
//	for(int i = 0; i < length; i++){
//		k = k < (length - 1) ? k + 1: 0;
//
//		targetPos.jointA = qcycle[k][0];
//		targetPos.jointB = qcycle[k][1];
//		targetPos.jointC = qcycle[k][2];
//
//		LegMoveToPositionWithDuration(leg, targetPos, dt);
//
//		usleep(dt * 1000000);
//	}
//	return;
//}

void LegTaskMovement(Leg *leg, float qcycle[][3], int length, int offset, float dt, int round, int inv) {

	int k = offset;
	int pre_k;
	Pos3DOF targetPos;
	Speed3DOF movingSpeed;

	targetPos.jointA = qcycle[k][0];
	targetPos.jointB = qcycle[k][1];
	targetPos.jointC = qcycle[k][2];
	LegMoveToPositionWithDuration(leg, targetPos, 2);
	sleep(2);

	for (int r = 0; r < round; r++) {
		for (int i = 0; i < length; i++) {
			float *target;
			float *pre;

			k = k < (length - 1) ? k + 1 : 0;
//			pre_k = k == 0 ? length - 1 : k - 1;
			pre_k = k == 0 ? 0 : k - 1;

			if (!inv) {
				target = qcycle[k];
				pre = qcycle[pre_k];
			} else {
				target = qcycle[(length - 1) - k];
				pre = qcycle[(length - 1) - pre_k];
			}

			movingSpeed.jointA = _diff(target[0], pre[0]) / dt;
			movingSpeed.jointB = _diff(target[1], pre[1]) / dt;
			movingSpeed.jointC = _diff(target[2], pre[2]) / dt;

			LegSetSpeedDeg(leg, movingSpeed);

			targetPos.jointA = target[0] + (target[0] - pre[0]) * 10.0;
			targetPos.jointB = target[1] + (target[1] - pre[1]) * 10.0;
			targetPos.jointC = target[2] + (target[2] - pre[2]) * 10.0;

			LegSetPositionDeg(leg, targetPos);

			usleep(dt * 1000000);
		}
	}
	return;
}

float _diff(float A, float B){
	if(A > B)
		return A - B;
	else
		return B - A;
}
