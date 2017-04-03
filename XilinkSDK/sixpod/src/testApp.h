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

volatile bool mpuInterruptTest = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReadyTest(XGpioPs* cbRef, u32 bank, u32 status) {
	mpuInterruptTest = true;
}

void prinfloat(float fval){
	int whole = fval;
	int thousandths = (fval - whole) * 1000;
	thousandths = thousandths > 0 ? thousandths : -thousandths;
	xil_printf("%d.%3d", whole, thousandths);
}

void testIMU() {
	int intrPin = 12;
	MPU6050 mpuTest;
	uint8_t fifoBuffer[64];
	// orientation/motion vars
	Quaternion q;           // [w, x, y, z]         quaternion container
	VectorInt16 aa;         // [x, y, z]            accel sensor measurements
	VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
	VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
	VectorFloat gravity;    // [x, y, z]            gravity vector
	float euler[3];         // [psi, theta, phi]    Euler angle container
	float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

	int status;
	ImuSetup(mpuTest, intrPin, dmpDataReadyTest);

	while (1) {
		while (!mpuInterruptTest) {
			//
		}
		mpuInterruptTest = false;
		status = ImuReadFifoBuffer(mpuTest, fifoBuffer);
		if (status == XST_SUCCESS) {
			mpuTest.dmpGetQuaternion(&q, fifoBuffer);
			mpuTest.dmpGetEuler(euler, &q);
			mpuTest.dmpGetGravity(&gravity, &q);
			mpuTest.dmpGetYawPitchRoll(ypr, &q, &gravity);
			mpuTest.dmpGetAccel(&aa, fifoBuffer);
			mpuTest.dmpGetLinearAccel(&aaReal, &aa, &gravity);
			mpuTest.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);

			xil_printf("quat: ");
			prinfloat(q.w);
			xil_printf(", ");
			prinfloat(q.x);
			xil_printf(", ");
			prinfloat(q.y);
			xil_printf(", ");
			prinfloat(q.z);
			xil_printf("\t\t");

			xil_printf("euler: ");
			prinfloat(euler[0] * 180 / M_PI);
			xil_printf(", ");
			prinfloat(euler[1] * 180 / M_PI);
			xil_printf(", ");
			prinfloat(euler[2] * 180 / M_PI);
			xil_printf("\t\t");

			xil_printf("ypr: ");
			prinfloat(ypr[0] * 180 / M_PI);
			xil_printf(", ");
			prinfloat(ypr[1] * 180 / M_PI);
			xil_printf(", ");
			prinfloat(ypr[2] * 180 / M_PI);
			xil_printf("\t\t");

			xil_printf("areal: ");
			prinfloat(aaReal.x);
			xil_printf(", ");
			prinfloat(aaReal.y);
			xil_printf(", ");
			prinfloat(aaReal.z);
			xil_printf("\t\t");

			xil_printf("aWorld: ");
			prinfloat(aaWorld.x);
			xil_printf(", ");
			prinfloat(aaWorld.y);
			xil_printf(", ");
			prinfloat(aaWorld.z);
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
