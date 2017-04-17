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

static QueueHandle_t xTrajQueue[6];
static TaskHandle_t xInitTask;

HEXAPOD HexapodRobot;

/* Task function */
#include "taskFunction.h"

static void init( void * );

int main (void) {
	BaseType_t status;

	for(int i = 0; i < 6; i++){
		xTrajQueue[i] = xQueueCreate( 20, sizeof( Trajectory3d ));
	}

	status = xTaskCreate( init, 			/* The function that implements the task. */
				( const char * ) "init", 	/* Text name for the task, provided to assist debugging only. */
				configMINIMAL_STACK_SIZE, 	/* The stack allocated to the task. */
				NULL, 						/* The task parameter is not used, so set to NULL. */
				tskIDLE_PRIORITY,			/* The task runs at the idle priority. */
				&xInitTask );

	if(status != pdPASS){
		xil_printf("Can not create init task.");
		return XST_FAILURE;
	}

	vTaskStartScheduler();

	return XST_SUCCESS;
}

static void init( void *pvParameters ) {
	BaseType_t status;
	TaskHandle_t xMovingTask;
	TaskHandle_t xWalkingTask;

	xil_printf("Initial\r\n");

	status = xTaskCreate( hexapodMovingTask,
				 ( const char * ) "Moving",
				 configMINIMAL_STACK_SIZE,
				 NULL,
				 tskIDLE_PRIORITY + 4,
				 &xMovingTask );

	if(status != pdPASS){
		xil_printf("Can not create Moving task.\r\n");
	}

	status = xTaskCreate( hexapodWalkingTask,
				 ( const char * ) "Walking",
				 configMINIMAL_STACK_SIZE,
				 NULL,
				 tskIDLE_PRIORITY,
				 &xWalkingTask );

	if(status != pdPASS){
		xil_printf("Can not create Walking task.\r\n");
	}

	xil_printf("Initialed\r\n");
	vTaskDelete(NULL);
	for(;;);
}


#endif //else if GEN_TEST_APP == 0

