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

void testIIC() {
	xil_printf("Test IIC with DS1307RTC.\r\n");
	int DS1307_ID = 0x68;

	u8 dataRead[7];

	u8 dataWrite[] = { 	0x00,		//DoW
						0x59,		//Sec
						0x20,		//Min
						0x02,		//Hour
						0x15,		//Day
						0x02,		//Month
						0x27		//Year
					};
	i2cWrite(DS1307_ID, 0, 7, dataWrite);

	while (1) {
		i2cRead(0x68, 0, 7, dataRead);
		xil_printf("Read Data : %x\t", dataRead[3]);
		xil_printf("%x/%x/%x\t", dataRead[4], dataRead[5], dataRead[6]);
		xil_printf("%x:%x:%x\r\n", dataRead[2], dataRead[1], dataRead[0]);
		sleep(1);
	}
}

void intrHandle(XGpioPs *callBackRef, u32 Bank, u32 Status) {
	xil_printf("Interrupt Occur with bank %x and status is %x\r\n", Bank,
			Status);
}

void testIntrModule() {
	int GPIOPS_PIN = 50;	//MIO50 button on ZYBO board.
	xil_printf("Test GpioPs Interrupt.\r\n");
	xil_printf("Press MIO50 button for generate interrupt.\r\n");

	//Setup interrupt with rising edge.
	setupIntrSystem(GPIOPS_PIN, intrHandle, INTR_TYPE_EDGE_RISING);

	//Enable interrupt.
	enableIntr(GPIOPS_PIN);

	while (1)
		;	//Wait for interrupt.
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

