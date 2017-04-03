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
	}
	Imu.setup(MPU_INTR_PIN);
}

void HEXAPOD::begin(){
	int status;
	float setup_time = 2.0;
	for(int i = 0; i < 6; i++){
		this->leg[i].moveTo(this->targetFootTip[i], setup_time);
	}
	sleep(3);
	for(int cntloop = 0; cntloop < 2000; ){
		while (!Imu.available());
		status = Imu.readFifoBuffer();
		if (status == XST_SUCCESS) {
			cntloop++;
		}
	}
	bodyRotOffset.y = Imu.euler[0];
	bodyRotOffset.p = Imu.euler[1];
	bodyRotOffset.r = Imu.euler[2];
}

void HEXAPOD::readIMU(){
	if (Imu.available()){
		int status = Imu.readFifoBuffer();
		if (status == XST_SUCCESS) {
			bodyRot.y = Imu.euler[0];
			bodyRot.p = Imu.euler[1];
			bodyRot.r = Imu.euler[2];

			bodyRot = bodyRot - bodyRotOffset;
			logBodyRot();
		}
	}
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
