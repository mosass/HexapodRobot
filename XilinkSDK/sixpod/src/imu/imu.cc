/*
 * imu.cc
 *
 *  Created on: Apr 4, 2017
 *      Author: Phanomphon Yotchon
 */

#include "MPU6050_6Axis_MotionApps20.h"
#include "imu.h"
#include "interrupt.h"

static volatile bool MpuIntrFlag;     // indicates whether MPU interrupt pin has gone high

static void dmpDataReady(XGpioPs* cbRef, u32 bank, u32 status) {
	MpuIntrFlag = true;
}

static MPU6050 mpuInts;

IMU::IMU(){
	MpuIntrFlag = false;
}

bool IMU::available(){
	return MpuIntrFlag;
}

int IMU::autoCalibrateOffset(int16_t &off_acc_x,
		int16_t &off_acc_y, int16_t &off_acc_z, int16_t &off_gyr_x,
		int16_t &off_gyr_y, int16_t &off_gyr_z){

	int16_t acc_x, acc_y, acc_z;
	int16_t gyr_x, gyr_y, gyr_z;

	off_acc_x = off_acc_y = off_acc_z = 0;
	off_gyr_x = off_gyr_y = off_gyr_z = 0;

	mpuInts.setXAccelOffset(off_acc_x);
	mpuInts.setYAccelOffset(off_acc_y);
	mpuInts.setZAccelOffset(off_acc_z);
	mpuInts.setXGyroOffset(off_gyr_x);
	mpuInts.setYGyroOffset(off_gyr_y);
	mpuInts.setZGyroOffset(off_gyr_z);

	int16_t e_acc_x, e_acc_y, e_acc_z;
	int16_t e_gyr_x, e_gyr_y, e_gyr_z;

	u8 v_acc_x, v_acc_y, v_acc_z;
	u8 v_gyr_x, v_gyr_y, v_gyr_z;
	v_acc_x = v_acc_y = v_acc_z = 0;
	v_gyr_x = v_gyr_y = v_gyr_z = 0;

	bool vaild = true;
	u8 cnt_loop = 0;

	do {
		cnt_loop++;
		vaild = true;
		mpuInts.getAcceleration(&acc_x, &acc_y, &acc_z);
		mpuInts.getRotation(&gyr_x, &gyr_y, &gyr_z);

		e_acc_x = (0 - acc_x) * 0.1;
		e_acc_y = (0 - acc_y) * 0.1;
		e_acc_z = (16384 - acc_z) * 0.1;

		e_gyr_x = (0 - gyr_x) * 0.1;
		e_gyr_y = (0 - gyr_y) * 0.1;
		e_gyr_z = (0 - gyr_z) * 0.1;

		if (v_acc_x < 5) {
			if (e_acc_x < 10) {
				v_acc_x++;
			} else {
				v_acc_x = 0;
			}
			off_acc_x += e_acc_x;
			mpuInts.setXAccelOffset(off_acc_x);
			vaild = false;
		}

		if (v_acc_y < 5) {
			if (e_acc_y < 10) {
				v_acc_y++;
			} else {
				v_acc_y = 0;
			}
			off_acc_y += e_acc_y;
			mpuInts.setYAccelOffset(off_acc_y);
			vaild = false;
		}

		if (v_acc_z < 5) {
			if (e_acc_z < 10) {
				v_acc_z++;
			} else {
				v_acc_z = 0;
			}
			off_acc_z += e_acc_z;
			mpuInts.setZAccelOffset(off_acc_z);
			vaild = false;
		}

		if (v_gyr_x < 5) {
			if (e_gyr_x < 10) {
				v_gyr_x++;
			} else {
				v_gyr_x = 0;
			}
			off_gyr_x += e_gyr_x;
			mpuInts.setXGyroOffset(off_gyr_x);
			vaild = false;
		}

		if (v_gyr_y < 5) {
			if (e_gyr_y < 10) {
				v_gyr_y++;
			} else {
				v_gyr_y = 0;
			}
			off_gyr_y += e_gyr_y;
			mpuInts.setYGyroOffset(off_gyr_y);
			vaild = false;
		}

		if (v_gyr_z < 5) {
			if (e_gyr_z < 10) {
				v_gyr_z++;
			} else {
				v_gyr_z = 0;
			}
			off_gyr_z += e_gyr_z;
			mpuInts.setZGyroOffset(off_gyr_z);
			vaild = false;
		}
	} while (cnt_loop < 100 && !vaild);

	if (cnt_loop == 100) {
		return XST_FAILURE;
	} else {
		return XST_SUCCESS;
	}
}

int IMU::setup (int intrPin){
	int status;
	int16_t ofs_acc_x, ofs_acc_y, ofs_acc_z;
	int16_t ofs_gyr_x, ofs_gyr_y, ofs_gyr_z;

	mpuInts.initialize();
	if(!mpuInts.testConnection()){
		return XST_FAILURE;
	}

	if(autoCalibrateOffset(ofs_acc_x, ofs_acc_y, ofs_acc_z, ofs_gyr_x, ofs_gyr_y, ofs_gyr_z)){
		return XST_FAILURE;
	}

	status = mpuInts.dmpInitialize();

	// supply your own gyro offsets here, scaled for min sensitivity
	mpuInts.setXAccelOffset(ofs_acc_x);
	mpuInts.setYAccelOffset(ofs_acc_y);
	mpuInts.setZAccelOffset(ofs_acc_z);

	mpuInts.setXGyroOffset(ofs_gyr_x);
	mpuInts.setYGyroOffset(ofs_gyr_y);
	mpuInts.setZGyroOffset(ofs_gyr_z);

	if(status == 0){
		mpuInts.setDMPEnabled(true);
		setupIntrSystem(intrPin, dmpDataReady, INTR_TYPE_EDGE_RISING);
		enableIntr(intrPin);
	} else {
		return XST_FAILURE;
	}
	MpuIntrFlag = false;
	return XST_SUCCESS;
}

int IMU::readFifoBuffer() {
	u8 mpuIntrStatus;
	u16 fifoCount;
	u16 packetSize = mpuInts.dmpGetFIFOPacketSize();

	mpuIntrStatus = mpuInts.getIntStatus();
	fifoCount = mpuInts.getFIFOCount();
	if ((mpuIntrStatus & 0x10) || fifoCount == 1024) {
		// FIFO overflow!
		mpuInts.resetFIFO();
		return XST_FAILURE;
	} else if (mpuIntrStatus & 0x02) {
		while (fifoCount < packetSize) {
			fifoCount = mpuInts.getFIFOCount();
		}

		mpuInts.getFIFOBytes(fifoBuffer, packetSize);
		fifoCount -= packetSize;

		mpuInts.dmpGetQuaternion(&quat, fifoBuffer);
		mpuInts.dmpGetEuler(euler, &quat);
		mpuInts.dmpGetGravity(&gravity, &quat);
		mpuInts.dmpGetYawPitchRoll(ypr, &quat, &gravity);
		mpuInts.dmpGetAccel(&accel, fifoBuffer);
		mpuInts.dmpGetLinearAccel(&accelReal, &accel, &gravity);
		mpuInts.dmpGetLinearAccelInWorld(&accelWorld, &accelReal, &quat);

		MpuIntrFlag = false;
		return XST_SUCCESS;
	}
	return XST_FAILURE;
}





