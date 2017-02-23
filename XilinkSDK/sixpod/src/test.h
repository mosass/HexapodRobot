/*
 * test.c
 *
 *  Created on: Feb 20, 2017
 *      Author: Phanomphon Yotchon
 */
/* Module includes */
#include "leg.h"
#include "qcycle.h"
#include "interrupt.h"
#include "i2c.h"

void testIIC(){
	xil_printf("Test IIC.\r\n");

	u8 dataread[7];

	i2cRead(0x68, 0, 7, dataread);
	xil_printf("Read Data : %x\t", dataread[3]);
	xil_printf("%x/%x/%x\t", dataread[4], dataread[5], dataread[6]);
	xil_printf("%x:%x:%x\r\n", dataread[2], dataread[1], dataread[0]);
	sleep(1);

	u8 date[] = {
			0x00,
			0x59,
			0x20,
			0x02,
			0x15,
			0x02,
			0x27
	};
	i2cWrite(0x68, 0, 7, date);

	while(1){
		i2cRead(0x68, 0, 7, dataread);
		xil_printf("Read Data : %x\t", dataread[3]);
		xil_printf("%x/%x/%x\t", dataread[4], dataread[5], dataread[6]);
		xil_printf("%x:%x:%x\r\n", dataread[2], dataread[1], dataread[0]);
		sleep(1);
	}
}

void intrHandle(XGpioPs *callBackRef, u32 Bank, u32 Status){
	xil_printf("%x\t%x\r\n", Bank, Status);
}

void testIntrModule(){
	xil_printf("test interrupt module\r\n");
	setupIntrSystem(50, intrHandle, INTR_TYPE_EDGE_RISING);
	enableIntr(50);
	while(1);
}

void testLegModule() {
	Leg leg;
	LegInitial(&leg, 1);

	leg.inverseA = FALSE;
	leg.inverseC = TRUE;

	LegTaskMovement(&leg, QCYCLE, QCYCLE_LENGTH, 0, 0.01, 2, FALSE);

	leg.inverseA = FALSE;

	LegTaskMovement(&leg, QCYCLE, QCYCLE_LENGTH, 0, 0.001, 2, TRUE);

	return;
}

