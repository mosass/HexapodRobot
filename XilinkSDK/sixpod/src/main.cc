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
#include "interrupt.h"
#include "MPU6050/imu.h"

#include "testApp.h"

MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

#define INTERRUPT_PIN 12
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady(XGpioPs* cbRef, u32 bank, u32 status) {
    mpuInterrupt = true;
}

void prinfloat(float fval){
	int whole = fval;
	int thousandths = (fval - whole) * 1000;
	thousandths = thousandths > 0 ? thousandths : -thousandths;
	xil_printf("%d.%d", whole, thousandths);
}

int main(void) {
	xil_printf("Test\r\n");
	mpu.initialize();
	xil_printf("initialize MPU\r\n");
	xil_printf("Test MPU connection\r\n");
	mpu.testConnection() ? xil_printf("MPU6050 connection successful\r\n") : xil_printf("MPU6050 connection failed\r\n");

//	int16_t acc_x, acc_y, acc_z;
//	int16_t gyr_x, gyr_y, gyr_z;
////	int n = -1600;
//
//	sleep(2);
//
//	while(1){
//
//		mpu.getAcceleration(&acc_x, &acc_y, &acc_z);
//		xil_printf("acc: ");
//		xil_printf("%3d\t", acc_x);
//		xil_printf("%3d\t", acc_y);
//		xil_printf("%3d\t", acc_z);
//		xil_printf("\t");
//
//		mpu.getRotation(&gyr_x, &gyr_y, &gyr_z);
//		xil_printf("gyr: ");
//		xil_printf("%3d\t", gyr_x);
//		xil_printf("%3d\t", gyr_y);
//		xil_printf("%3d\t", gyr_z);
//		xil_printf("\t");
//
////		if(acc_z > -16384){
////			n--;
////		}
////		else if(acc_z < -16384){
////			n++;
////		}
////		mpu.setXAccelOffset(n);
////		mpu.setYAccelOffset(n);
////		mpu.setZAccelOffset(n);
//
////		mpu.setXGyroOffset(n);
////		mpu.setYGyroOffset(n);
////		mpu.setZGyroOffset(n);
//
////		xil_printf("n: ");
////		xil_printf("%d", n);
//		xil_printf("\r\n");
//		usleep(100000);
//	}

	// load and configure the DMP
	xil_printf("Initializing DMP...\r\n");
	devStatus = mpu.dmpInitialize();

	// supply your own gyro offsets here, scaled for min sensitivity
	mpu.setXAccelOffset(-3645);
	mpu.setYAccelOffset(-50);
	mpu.setZAccelOffset(1935);

	mpu.setXGyroOffset(-32);
	mpu.setYGyroOffset(18);
	mpu.setZGyroOffset(7);

//	int t = mpu.getDeviceID();
//	xil_printf("Device ID : %x\r\n", t);
//	return 1;

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
    	xil_printf("Enabling DMP...\r\n");
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        xil_printf("Enabling interrupt detection (Arduino external interrupt 0)...\r\n");
        setupIntrSystem(INTERRUPT_PIN, dmpDataReady, INTR_TYPE_EDGE_RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        xil_printf("DMP ready! Waiting for first interrupt...\r\n");
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
    	xil_printf("DMP Initialization failed (code %d)\r\n", devStatus);
    }

    enableIntr(12);

    while(1){
    	if (!dmpReady) return XST_FAILURE;
    	// wait for MPU interrupt or extra packet(s) available
		while (!mpuInterrupt && fifoCount < packetSize) {
			// other program behavior stuff here
			// .
			// .
			// .
			// if you are really paranoid you can frequently test in between other
			// stuff to see if mpuInterrupt is true, and if so, "break;" from the
			// while() loop to immediately process the MPU data
			// .
			// .
			// .
		}

		// reset interrupt flag and get INT_STATUS byte
		mpuInterrupt = false;
		mpuIntStatus = mpu.getIntStatus();

		// get current FIFO count
		fifoCount = mpu.getFIFOCount();

		// check for overflow (this should never happen unless our code is too inefficient)
		if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
			// reset so we can continue cleanly
			mpu.resetFIFO();
			xil_printf("FIFO overflow!\r\n");
			// otherwise, check for DMP data ready interrupt (this should happen frequently)
		} else if (mpuIntStatus & 0x02) {
			// wait for correct available data length, should be a VERY short wait
			while (fifoCount < packetSize)
				fifoCount = mpu.getFIFOCount();

			// read a packet from FIFO
			mpu.getFIFOBytes(fifoBuffer, packetSize);

			// track FIFO count here in case there is > 1 packet available
			// (this lets us immediately read more without waiting for an interrupt)
			fifoCount -= packetSize;

			mpu.dmpGetQuaternion(&q, fifoBuffer);
			mpu.dmpGetEuler(euler, &q);
			mpu.dmpGetGravity(&gravity, &q);
			mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
			mpu.dmpGetAccel(&aa, fifoBuffer);
			mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
			mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);

			xil_printf("quat: ");
			prinfloat(q.w);
			xil_printf("\t");
			prinfloat(q.x);
			xil_printf("\t");
			prinfloat(q.y);
			xil_printf("\t");
			prinfloat(q.z);
			xil_printf("\t");


			xil_printf("euler: ");
			prinfloat(euler[0] * 180/M_PI);
			xil_printf("\t");
			prinfloat(euler[1] * 180 / M_PI);
			xil_printf("\t");
			prinfloat(euler[2] * 180 / M_PI);
			xil_printf("\t");


			xil_printf("ypr: ");
			prinfloat(ypr[0] * 180/M_PI);
			xil_printf("\t");
			prinfloat(ypr[1] * 180 / M_PI);
			xil_printf("\t");
			prinfloat(ypr[2] * 180 / M_PI);
			xil_printf("\t");

//			xil_printf("areal: ");
//			prinfloat(aaReal.x);
//			xil_printf("\t");
//			prinfloat(aaReal.y);
//			xil_printf("\t");
//			prinfloat(aaReal.z);
//			xil_printf("\t");

//			xil_printf("aWorld: ");
//			prinfloat(aaWorld.x);
//			xil_printf("\t");
//			prinfloat(aaWorld.y);
//			xil_printf("\t");
//			prinfloat(aaWorld.z);
//			xil_printf("\t");

			xil_printf("\r\n");
		}
	}


	return XST_SUCCESS;
}
