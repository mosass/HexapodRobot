/*
 * hexapod.h
 *
 *  Created on: Apr 2, 2017
 *      Author: Phanomphon Yotchon
 */

#ifndef SRC_HEXAPOD_HEXAPOD_H_
#define SRC_HEXAPOD_HEXAPOD_H_

#include "stdint.h"
#include "leg.h"
#include "hexapod_type.h"
#include "math.h"
#include "hexapod_config.h"
#include "imu.h"

typedef struct trajectory3d {
	float x;
	float y;
	float z;
	float duration;
} Trajectory3d;

class HEXAPOD {
private:

public:
	Leg leg[6];
	FootTip footTip[6];
	FootTip targetFootTip[6];
	Rot3d bodyRot;
	Rot3d bodyRotOffset;
	Rot3d bodyRotTarget;

	bool improveYaw;
	bool improvePitch;
	bool improveRoll;

	float stepTime;
	float dt;

	HEXAPOD();
	void begin();
	bool readIMU();
	void logBodyRot();
	bool balance();
	void moving();

	void applyRotToGait(int legId, Trajectory3d& Traj);
	void updateGoalPosition();
};

static HEXAPOD Hexapod;

#endif /* SRC_HEXAPOD_HEXAPOD_H_ */
