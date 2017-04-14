/*
 * hexapod.cc
 *
 *  Created on: Apr 2, 2017
 *      Author: Phanomphon Yotchon
 */

#include "hexapod.h"
#include "sleep.h"

static void print_float(float f){
	int i = (int) f;
	int s = (f - i) * 1000;

	xil_printf("%d.", i);
	s = s < 0 ? -s : s;
	if(s < 100)
		xil_printf("0");
	if(s < 10)
		xil_printf("0");
	xil_printf("%d\t", s);
}

HEXAPOD::HEXAPOD(){
	for(int i = 0; i < 6; i++){
		this->leg[i].setup(i+1, INITIAL_FOOTTIP_Z_OFF);
		this->targetFootTip[i].x = INITIAL_FOOTTIP_X;
		this->targetFootTip[i].y = INITIAL_FOOTTIP_Y;
		this->targetFootTip[i].z = INITIAL_FOOTTIP_Z;

		this->footTip[i].x = INITIAL_FOOTTIP_X;
		this->footTip[i].y = INITIAL_FOOTTIP_Y;
		this->footTip[i].z = INITIAL_FOOTTIP_Z;
	}
	this->bodyRotTarget.y = INITIAL_ROT_Y;
	this->bodyRotTarget.p = INITIAL_ROT_P;
	this->bodyRotTarget.r = INITIAL_ROT_R;

	this->stepTime = INITIAL_STEP_TIME;
	this->dt = INITIAL_DT;

	this->improveYaw = true;
	this->improvePitch = true;
	this->improveRoll = true;
}

void HEXAPOD::begin(){
	Imu.setup(MPU_INTR_PIN);
	int status;
	float setup_time = SETUP_TIME;
	for(int i = 0; i < 6; i++){
		this->leg[i].moveTo(this->targetFootTip[i], setup_time);
	}
	sleep(3);
	for(int cntloop = 0; cntloop < 2000; ){
		while (!Imu.available());
		xil_printf(".");
		if(cntloop % 100 == 0)
			xil_printf("\r\n");
		status = Imu.readFifoBuffer();
		if (status == XST_SUCCESS) {
			cntloop++;
		}
	}
	bodyRotOffset.y = Imu.euler[0];
	bodyRotOffset.p = Imu.euler[1];
	bodyRotOffset.r = Imu.euler[2];
}

bool HEXAPOD::readIMU(){
	if (Imu.available()){
		int status = Imu.readFifoBuffer();
		if (status == XST_SUCCESS) {
			bodyRot.y = Imu.euler[0];
			bodyRot.p = Imu.euler[1];
			bodyRot.r = Imu.euler[2];

			bodyRot = bodyRot - bodyRotOffset;
			logBodyRot();
			return true;
		}
	}

	return false;
}

void HEXAPOD::logBodyRot(){
	xil_printf("current YPR : ");
	print_float(bodyRot.y);
	print_float(bodyRot.p);
	print_float(bodyRot.r);

	xil_printf("offset YPR : ");
	print_float(bodyRotOffset.y);
	print_float(bodyRotOffset.p);
	print_float(bodyRotOffset.r);

	xil_printf("\r\n");
}

bool HEXAPOD::balance(){
	float x, y, z;
	float s_ye = sin(bodyRotTarget.y - bodyRot.y);
	float s_pe = sin(bodyRotTarget.p - bodyRot.p);
	float s_re = sin(bodyRotTarget.r - bodyRot.r);

	Rot3d error = bodyRot.diff(bodyRotTarget);

	if(Rot3d::toDeg(error.y) < ALLOW_ROT_ERROR || !this->improveYaw){
		s_ye = 0;
	}

	if(Rot3d::toDeg(error.p) < ALLOW_ROT_ERROR || !this->improvePitch){
		s_pe = 0;
	}

	if(Rot3d::toDeg(error.r) < ALLOW_ROT_ERROR || !this->improveRoll){
		s_re = 0;
	}

	if(s_ye != 0 && s_pe != 0 && s_re != 0){
		return false;
	}

	//leg 1
	x = footTip[0].x;
	y = footTip[0].y;
	z = footTip[0].z;
	footTip[0].x = x - (H_YBODY + y)*s_ye - z*s_pe;
	footTip[0].y = y + (H_XBODY + x)*s_ye + z*s_re;
	footTip[0].z = z - (H_XBODY + x)*s_pe + (H_YBODY + y)*s_re;

	//leg 2
	x = footTip[1].x;
	y = footTip[1].y;
	z = footTip[1].z;
	footTip[1].x = x - (H_YBODY + y)*s_ye - z*s_pe;
	footTip[1].y = y + (x)*s_ye + z*s_re;
	footTip[1].z = z + (H_YBODY + y)*s_re;

	//leg 3
	x = footTip[2].x;
	y = footTip[2].y;
	z = footTip[2].z;
	footTip[2].x = x - (H_YBODY + y)*s_ye - z*s_pe;
	footTip[2].y = y - (H_XBODY - x)*s_ye + z*s_re;
	footTip[2].z = z + (H_XBODY - x)*s_pe + (H_YBODY + y)*s_re;

	//leg 4
	x = footTip[3].x;
	y = footTip[3].y;
	z = footTip[3].z;
	footTip[3].x = x + (H_YBODY + y)*s_ye - z*s_pe;
	footTip[3].y = y - (H_XBODY + x)*s_ye - z*s_re;
	footTip[3].z = z - (H_XBODY + x)*s_pe - (H_YBODY + y)*s_re;

	//leg 5
	x = footTip[4].x;
	y = footTip[4].y;
	z = footTip[4].z;
	footTip[4].x = x + (H_YBODY + y)*s_ye - z*s_pe;
	footTip[4].y = y - (x)*s_ye - z*s_re;
	footTip[4].z = z - (H_YBODY + y)*s_re;

	//leg 6
	x = footTip[5].x;
	y = footTip[5].y;
	z = footTip[5].z;
	footTip[5].x = x + (H_YBODY + y)*s_ye - z*s_pe;
	footTip[5].y = y + (H_XBODY - x)*s_ye - z*s_re;
	footTip[5].z = z + (H_XBODY - x)*s_pe - (H_YBODY + y)*s_re;

	return true;
}

void HEXAPOD::moving(){
	for(int i = 0; i < 6; i++){
		this->leg[i].moveTo(this->footTip[i], this->dt);
	}
	return;
}
