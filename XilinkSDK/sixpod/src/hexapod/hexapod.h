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

class HEXAPOD {
private:

public:
	Leg leg[6];
	FootTip targetFootTip[6];
	Rot3d bodyRot;
	Rot3d bodyRotOffset;

	HEXAPOD();
	void begin();
	void readIMU();
	void logBodyRot();
};

static HEXAPOD Hexapod;

#endif /* SRC_HEXAPOD_HEXAPOD_H_ */
