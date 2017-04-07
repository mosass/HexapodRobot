/*
 * Main C++ Application
 */

#define GEN_TEST_APP  0

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
/* Xilinx includes. */
#include "stdint.h"
#include "stdio.h"
#include "xil_printf.h"
#include "xparameters.h"
#include "sleep.h"
#include "xstatus.h"

#if GEN_TEST_APP == 1
#include "testApp.h"
int main (void) {
	xil_printf("TestHexapod\r\n");
	testHexapod();
	return XST_SUCCESS;
}
#elif GEN_TEST_APP == 0

/* Module includes */
#include "hexapod.h"

void prinfloat(float fval){
	int whole = fval;
	int thousandths = (fval - whole) * 1000;
	thousandths = thousandths > 0 ? thousandths : -thousandths;
	xil_printf("%d.%3d", whole, thousandths);
}

static void hexapodBalanceTask( void *pvParameters ){
	const TickType_t xsecond = pdMS_TO_TICKS( 1000 );
	vTaskDelay( xsecond );
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
			vTaskDelay( xsecond );
		}
//		vTaskDelay( xsecond );
	}
}

static void hexapodGaitTask( void *pvParameters ){
	const TickType_t xsecond = pdMS_TO_TICKS( 2000 );
	for(;;){
		xil_printf("A\r\n");
		vTaskDelay( xsecond );
	}
}

static TaskHandle_t xBalanceTask;
static TaskHandle_t xGaitTask;

int main (void) {
	xil_printf("Test\r\n");

	xTaskCreate( hexapodBalanceTask, 					/* The function that implements the task. */
				( const char * ) "Balance", 		/* Text name for the task, provided to assist debugging only. */
				configMINIMAL_STACK_SIZE, 	/* The stack allocated to the task. */
				NULL, 						/* The task parameter is not used, so set to NULL. */
				tskIDLE_PRIORITY,			/* The task runs at the idle priority. */
				&xBalanceTask );

	xTaskCreate( hexapodGaitTask,
				 ( const char * ) "Gait",
				 configMINIMAL_STACK_SIZE,
				 NULL,
				 tskIDLE_PRIORITY,
				 &xGaitTask );

	vTaskStartScheduler();

	for( ;; );

	return XST_SUCCESS;
}
#endif //else if GEN_TEST_APP == 0

