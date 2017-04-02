/*
 * leg.c
 *
 *  Created on: Feb 6, 2017
 *      Author: Phanomphon Yotchon
 */


#include "../hexapod_leg/leg.h"

#define RAD2DEG(Q) 			((Q) / M_PI) * 180

static float _diff(float A, float B) {
	if (A > B)
		return A - B;
	else
		return B - A;
}

void LegInitial(Leg *leg, int Id, FootTipPosition ft) {
	leg->legID = Id;

	// for all legs.
	leg->footTip = ft;
	leg->footTipTarget = ft;

	switch(Id) {
	case 1:
		leg->jointA = LEG1_ID_A;
		leg->jointB = LEG1_ID_B;
		leg->jointC = LEG1_ID_C;

		leg->inverseA = LEG1_ID_INVA;
		leg->inverseB = LEG1_ID_INVB;
		leg->inverseC = LEG1_ID_INVC;
		break;
	case 2:
		leg->jointA = LEG2_ID_A;
		leg->jointB = LEG2_ID_B;
		leg->jointC = LEG2_ID_C;

		leg->inverseA = LEG2_ID_INVA;
		leg->inverseB = LEG2_ID_INVB;
		leg->inverseC = LEG2_ID_INVC;
		break;
	case 3:
		leg->jointA = LEG3_ID_A;
		leg->jointB = LEG3_ID_B;
		leg->jointC = LEG3_ID_C;

		leg->inverseA = LEG3_ID_INVA;
		leg->inverseB = LEG3_ID_INVB;
		leg->inverseC = LEG3_ID_INVC;
		break;
	case 4:
		leg->jointA = LEG4_ID_A;
		leg->jointB = LEG4_ID_B;
		leg->jointC = LEG4_ID_C;

		leg->inverseA = LEG4_ID_INVA;
		leg->inverseB = LEG4_ID_INVB;
		leg->inverseC = LEG4_ID_INVC;
		break;
	case 5:
		leg->jointA = LEG5_ID_A;
		leg->jointB = LEG5_ID_B;
		leg->jointC = LEG5_ID_C;

		leg->inverseA = LEG5_ID_INVA;
		leg->inverseB = LEG5_ID_INVB;
		leg->inverseC = LEG5_ID_INVC;
		break;
	case 6:
		leg->jointA = LEG6_ID_A;
		leg->jointB = LEG6_ID_B;
		leg->jointC = LEG6_ID_C;

		leg->inverseA = LEG6_ID_INVA;
		leg->inverseB = LEG6_ID_INVB;
		leg->inverseC = LEG6_ID_INVC;
		break;
	default:
		leg->jointA = 1;
		leg->jointB = 2;
		leg->jointC = 3;

		leg->inverseA = FALSE;
		leg->inverseB = FALSE;
		leg->inverseC = FALSE;
	}

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

Pos3DOF LegIk(FootTipPosition ft, float z_off) {
	return LegIk(ft.x, ft.y, ft.z, z_off);
}

Pos3DOF LegIk(float x, float y, float z, float z_off){
	float C = COXA;
	float F = FEMUR;
	float T = TIBIA;

	Pos3DOF jvar;

	jvar.jointA = 150.0 + RAD2DEG((atan(x/y)));
	float L1 = sqrt(x*x + y*y);
	float L = sqrt((L1 - C)*(L1 - C) + (z_off - z)*(z_off - z));
	jvar.jointB = 150.0 + RAD2DEG(acos((F*F + L*L - T*T)/(2*L*F)) - atan((z_off - z)/(L1 - C)));
	jvar.jointC = 90.0 + RAD2DEG(acos((T*T + F*F - L*L)/(2*T*F)) - M_PI_2);

	return jvar;
}
