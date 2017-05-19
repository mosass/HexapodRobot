/*
 * leg.cc
 *
 *  Created on: Apr 3, 2017
 *      Author: Phanomphon Yotchon
 */

#include "hexapod_config.h"
#include "leg_type.h"
#include "math.h"
#include "joint_controls.h"

#include "leg.h"

inline float toDeg(float rad){
	return ((rad) / M_PI) * 180.0f;
}

void Leg::setup(int Id = 0, float z_off = INITIAL_FOOTTIP_Z_OFF){
	this->id = Id;
//	this->footTipPos = FootTip();
	this->invX = false;
	this->invY = false;
	this->invZ = false;
	this->zOffset = z_off;

	switch(Id){
	case 1:
		this->jointIdA = LEG1_ID_A;
		this->jointIdB = LEG1_ID_B;
		this->jointIdC = LEG1_ID_C;

		this->invA = LEG1_ID_INVA;
		this->invB = LEG1_ID_INVB;
		this->invC = LEG1_ID_INVC;
		break;
	case 2:
		this->jointIdA = LEG2_ID_A;
		this->jointIdB = LEG2_ID_B;
		this->jointIdC = LEG2_ID_C;

		this->invA = LEG2_ID_INVA;
		this->invB = LEG2_ID_INVB;
		this->invC = LEG2_ID_INVC;
		break;
	case 3:
		this->jointIdA = LEG3_ID_A;
		this->jointIdB = LEG3_ID_B;
		this->jointIdC = LEG3_ID_C;

		this->invA = LEG3_ID_INVA;
		this->invB = LEG3_ID_INVB;
		this->invC = LEG3_ID_INVC;
		break;
	case 4:
		this->jointIdA = LEG4_ID_A;
		this->jointIdB = LEG4_ID_B;
		this->jointIdC = LEG4_ID_C;

		this->invA = LEG4_ID_INVA;
		this->invB = LEG4_ID_INVB;
		this->invC = LEG4_ID_INVC;
		break;
	case 5:
		this->jointIdA = LEG5_ID_A;
		this->jointIdB = LEG5_ID_B;
		this->jointIdC = LEG5_ID_C;

		this->invA = LEG5_ID_INVA;
		this->invB = LEG5_ID_INVB;
		this->invC = LEG5_ID_INVC;
		break;
	case 6:
		this->jointIdA = LEG6_ID_A;
		this->jointIdB = LEG6_ID_B;
		this->jointIdC = LEG6_ID_C;

		this->invA = LEG6_ID_INVA;
		this->invB = LEG6_ID_INVB;
		this->invC = LEG6_ID_INVC;
		break;
	default:
		this->jointIdA = 1;
		this->jointIdB = 2;
		this->jointIdC = 3;

		this->invA = false;
		this->invB = false;
		this->invC = false;
	}
}

void Leg::move(bool skipSetSpeed = false){
	if(!skipSetSpeed){
		JointSetMovingSpeedDeg(this->jointIdA, this->linkSpeed.a);
		JointSetMovingSpeedDeg(this->jointIdB, this->linkSpeed.b);
		JointSetMovingSpeedDeg(this->jointIdC, this->linkSpeed.c);
	}

	if(this->invA)
		JointSetGoalPositionDeg(this->jointIdA, 300.0f - this->linkPos.a);
	else
		JointSetGoalPositionDeg(this->jointIdA, this->linkPos.a);

	if(this->invB)
		JointSetGoalPositionDeg(this->jointIdB, 300.0f - this->linkPos.b);
	else
		JointSetGoalPositionDeg(this->jointIdB, this->linkPos.b);

	if(this->invC)
		JointSetGoalPositionDeg(this->jointIdC, 300.0f - this->linkPos.c);
	else
		JointSetGoalPositionDeg(this->jointIdC, this->linkPos.c);

	return;
}

void Leg::setGoalPosition(FootTip& targetFootTipPos){
	this->footTipPos = targetFootTipPos;
	this->linkPos = this->calcIk();
	this->move(true); //ignore speed
	return;
}

/* **Only call this function when Motor is stationary.
 * because this function returned the wrong value if Motor is moving */
Link3d Leg::getPresentPosition(){
	Link3d pos;
	pos.a = JointGetPresentPositionDeg(this->jointIdA);
	pos.b = JointGetPresentPositionDeg(this->jointIdB);
	pos.c = JointGetPresentPositionDeg(this->jointIdC);

	if(this->invA)
		pos.a = 300.0f - pos.a;

	if(this->invB)
		pos.b = 300.0f - pos.b;

	if(this->invC)
		pos.c = 300.0f - pos.c;

	return pos;
}

void Leg::moveTo(FootTip& targetFootTipPos, float in_sec){
	Link3d currentJointPos = this->linkPos;
	this->footTipPos = targetFootTipPos;
	this->linkPos = this->calcIk();
	this->linkSpeed = this->linkPos.diff(currentJointPos) / in_sec;
	this->move();
	return;
}

Link3d Leg::calcIk(){
	float C = COXA;
	float F = FEMUR;
	float T = TIBIA;

	float x = this->footTipPos.x;
	float y = this->footTipPos.y;
	float z = this->footTipPos.z;
	float z_off = this->zOffset;

	Link3d result;

	result.a = 150.0f + toDeg(atan(x/y));
	float L1 = sqrt(x*x + y*y);
	float L = sqrt((L1 - C)*(L1 - C) + (z_off - z)*(z_off - z));
	result.b = 165.0f + toDeg(acos((F*F + L*L - T*T)/(2*L*F)) - atan((z_off - z)/(L1 - C)));
	result.c = 92.0f + toDeg(acos((T*T + F*F - L*L)/(2*T*F)) - M_PI_2);

	return result;
}





