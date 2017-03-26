/*
 * Empty C++ Application
 */

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
/* Xilinx includes. */
#include "stdio.h"
#include "xil_printf.h"
#include "xparameters.h"
#include "sleep.h"
/* Module includes */
#include "leg.h"
#include "imu.h"

#include "testApp.h"

Leg leg[6];

void initial6l() {
//	sleep(3);
	Pos3DOF targetPos;
	FootTipPosition ft;
	for(int i = 0; i < 6; i ++){
		LegInitial(&leg[i], i+1);

		xil_printf("Leg %d => \t", i+1);

		targetPos = LegIk(leg[i].footTip, 5);
		xil_printf("%d\t%d\t%d\r\n", (int)targetPos.jointA, (int)targetPos.jointB, (int)targetPos.jointC);

		LegMoveToPositionWithDuration(&leg[i], targetPos, 1);
	}
	sleep(3);
	for(int i = 0; i < 6; i ++){
		LegInitial(&leg[i], i+1);

		xil_printf("Leg : %d =>\t", i+1);

		targetPos = LegGetPresentPositionDeg(&leg[i]);
		xil_printf("%d\t%d\t%d\r\n", (int)targetPos.jointA, (int)targetPos.jointB, (int)targetPos.jointC);
	}

	return;
}


int main (void) {
	int ph = 0;
	initial6l();
	return 0;
	Pos3DOF targetPos;
	int legA1, legA2, legA3;
	int legB1, legB2, legB3;

	for(int g = 0; g < 20; g++){
		if(g % 2 == 0){
			legA1 = 0;
			legA2 = 2;
			legA3 = 4;

			legB1 = 1;
			legB2 = 3;
			legB3 = 5;
		}else {
			legA1 = 1;
			legA2 = 3;
			legA3 = 5;

			legB1 = 0;
			legB2 = 2;
			legB3 = 4;
		}

		switch(ph){
		case 0:
			leg[legA1].footTip.z = 3;
			leg[legA2].footTip.z = 3;
			leg[legA3].footTip.z = 3;

			leg[legB1].footTip.z = 0;
			leg[legB2].footTip.z = 0;
			leg[legB3].footTip.z = 0;
			break;
		case 1:
			leg[legA1].footTip.x = -5;
			leg[legA2].footTip.x = -5;
			leg[legA3].footTip.x = -5;

			leg[legB1].footTip.x = 5;
			leg[legB2].footTip.x = 5;
			leg[legB3].footTip.x = 5;
			break;
		case 2:
			leg[legA1].footTip.z = 0;
			leg[legA2].footTip.z = 0;
			leg[legA3].footTip.z = 0;

			leg[legB1].footTip.z = 0;
			leg[legB2].footTip.z = 0;
			leg[legB3].footTip.z = 0;
			break;
		default:
			break;
		}


		ph = (ph + 1) % 3;

		for(int i = 0; i < 6; i++){
			LegMoveToPositionWithDuration(&leg[i], LegIk(leg[i].footTip, 5), 1);
		}
		sleep(1);
	}

	return XST_SUCCESS;
}

