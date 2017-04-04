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
#include "hexapod.h"

void prinfloat(float fval){
	int whole = fval;
	int thousandths = (fval - whole) * 1000;
	thousandths = thousandths > 0 ? thousandths : -thousandths;
	xil_printf("%d.%3d", whole, thousandths);
}

void testHexapod(){
	Hexapod.begin();
	while(1){
		if(Hexapod.readIMU()){
			if(Hexapod.balance()){
				Hexapod.moving();
				usleep(Hexapod.time_step * 1000 * 1000);
			}
		}
	}
}

void testIMU() {
	int status;
	int intrPin = 12;
	Imu.setup(intrPin);

	while (1) {
		while (!Imu.available()) {
//			xil_printf("Wait for intr\r\n");
		}
		status = Imu.readFifoBuffer();
		if (status == XST_SUCCESS) {
			xil_printf("quat: ");
			prinfloat(Imu.quat.w);
			xil_printf(", ");
			prinfloat(Imu.quat.x);
			xil_printf(", ");
			prinfloat(Imu.quat.y);
			xil_printf(", ");
			prinfloat(Imu.quat.z);
			xil_printf("\t\t");

			xil_printf("euler: ");
			prinfloat(Imu.euler[0] * 180 / M_PI);
			xil_printf(", ");
			prinfloat(Imu.euler[1] * 180 / M_PI);
			xil_printf(", ");
			prinfloat(Imu.euler[2] * 180 / M_PI);
			xil_printf("\t\t");

			xil_printf("ypr: ");
			prinfloat(Imu.ypr[0] * 180 / M_PI);
			xil_printf(", ");
			prinfloat(Imu.ypr[1] * 180 / M_PI);
			xil_printf(", ");
			prinfloat(Imu.ypr[2] * 180 / M_PI);
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
	Leg L[6];
	float z_off = 7;
	float in_sec = 2;
	FootTip targetFt(-5, 14, 0);

	for(int i = 0; i < 6; i++){
		L[i].setup(i+1, z_off);
		L[i].moveTo(targetFt, in_sec);
	}

	while(1){
		Link3d lp = L[0].getPresentPosition();

		xil_printf("%d %d %d\r\n", (int)lp.a, (int)lp.b, (int)lp.c);
		sleep(1);
	}
	return;
}

#endif // GEN_TEST_APP == 1
