/*
 * test.c
 *
 *  Created on: Feb 20, 2017
 *      Author: Phanomphon Yotchon
 */
/* Module includes */

#if GEN_TEST_APP == 1

#include "leg.h"
#include "interrupt.h"
#include "i2c.h"
#include "imu.h"

void prinfloat(float fval){
	int whole = fval;
	int thousandths = (fval - whole) * 1000;
	thousandths = thousandths > 0 ? thousandths : -thousandths;
	xil_printf("%d.%3d", whole, thousandths);
}

void testIMU() {
	int status;
	int intrPin = 12;
	IMU imu;
	imu.setup(intrPin);

	while (1) {
		while (!MpuIntrFlag) {
			//
		}
		status = imu.readFifoBuffer();
		if (status == XST_SUCCESS) {
			xil_printf("quat: ");
			prinfloat(imu.quat.w);
			xil_printf(", ");
			prinfloat(imu.quat.x);
			xil_printf(", ");
			prinfloat(imu.quat.y);
			xil_printf(", ");
			prinfloat(imu.quat.z);
			xil_printf("\t\t");

			xil_printf("euler: ");
			prinfloat(imu.euler[0] * 180 / M_PI);
			xil_printf(", ");
			prinfloat(imu.euler[1] * 180 / M_PI);
			xil_printf(", ");
			prinfloat(imu.euler[2] * 180 / M_PI);
			xil_printf("\t\t");

			xil_printf("ypr: ");
			prinfloat(imu.ypr[0] * 180 / M_PI);
			xil_printf(", ");
			prinfloat(imu.ypr[1] * 180 / M_PI);
			xil_printf(", ");
			prinfloat(imu.ypr[2] * 180 / M_PI);
			xil_printf("\r\n");
		}
	}

	return;
}

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
	Leg L1(1);
	Leg L2(2);
	Leg L3(3);
	Leg L4(4);
	Leg L5(5);
	Leg L6(6);

	L1.zOffset = 7;
	L2.zOffset = 7;
	L3.zOffset = 7;
	L4.zOffset = 7;
	L5.zOffset = 7;
	L6.zOffset = 7;

	FootTip targetFt(-5, 14, 0);
	L1.moveTo(targetFt, 2);
	L2.moveTo(targetFt, 2);
	L3.moveTo(targetFt, 2);
	L4.moveTo(targetFt, 2);
	L5.moveTo(targetFt, 2);
	L6.moveTo(targetFt, 2);

	while(1){
		Link3d lp = L1.getPresentPosition();

		xil_printf("%d %d %d\r\n", (int)lp.a, (int)lp.b, (int)lp.c);
		sleep(1);
	}
	return;
}

#endif // GEN_TEST_APP == 1
