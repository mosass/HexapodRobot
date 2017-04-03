/*
 * imu.h
 *
 *  Created on: Feb 27, 2017
 *      Author: Phanomphon Yotchon
 */

#include "interrupt.h"
#include "helper_3dmath.h"

#ifndef SRC_IMU_IMU_H_
#define SRC_IMU_IMU_H_

class IMU {
public:
	uint8_t fifoBuffer[64];
	Quaternion quat;           // [w, x, y, z]         quaternion container
	VectorInt16 accel;         // [x, y, z]            accel sensor measurements
	VectorInt16 accelReal;     // [x, y, z]            gravity-free accel sensor measurements
	VectorInt16 accelWorld;    // [x, y, z]            world-frame accel sensor measurements
	VectorFloat gravity;    // [x, y, z]            gravity vector
	float euler[3];         // [psi, theta, phi]    Euler angle container
	float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

	IMU();

	int autoCalibrateOffset(int16_t &off_acc_x,
			int16_t &off_acc_y, int16_t &off_acc_z, int16_t &off_gyr_x,
			int16_t &off_gyr_y, int16_t &off_gyr_z);
	int setup (int intrPin);
	int readFifoBuffer();
	bool available();
};

static IMU Imu;

#endif /* SRC_IMU_IMU_H_ */
