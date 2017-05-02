/*
 * Main C++ Application
 */

#define GEN_TEST_APP  0

/* FreeRTOS includes. */
#include "platform.h"

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
static TaskHandle_t xMovingTask;
static TaskHandle_t xWalkingTask;
static TaskHandle_t xLegGait[6];

HEXAPOD HexapodRobot;

/* Task function */
#include "networkTaskFunction.h"
#include "taskFunction.h"

extern "C" void lwip_init();
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
				DEFAULT_THREAD_PRIO,			/* The task runs at the idle priority. */
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

	xil_printf("Initial\r\n");

	Hexapod.begin();

	xil_printf("Initial Network\r\n");
	/* initialize lwIP before calling sys_thread_new */
//	lwip_init();

	/* any thread using lwIP should be created using sys_thread_new */
//	sys_thread_new("NW_THRD", network_thread, NULL,
//			THREAD_STACKSIZE, DEFAULT_THREAD_PRIO);

	TickType_t st = pdMS_TO_TICKS( 3000 );
	vTaskDelay( st );
	xil_printf("Initialed Network\r\n");

	status = xTaskCreate( hexapodMovingTask,
				 ( const char * ) "Moving",
				 configMINIMAL_STACK_SIZE,
				 NULL,
				 DEFAULT_THREAD_PRIO,
				 &xMovingTask );

	if(status != pdPASS){
		xil_printf("Can not create Moving task.\r\n");
	}

	const char * taskName[6] = {
			"Leg1Gait",
			"Leg2Gait",
			"Leg3Gait",
			"Leg4Gait",
			"Leg5Gait",
			"Leg6Gait"
	};

	for(int i = 0; i < 6; i++){
		xQueueReset(xTrajQueue[i]);
		xTaskCreate( hexapodLegGaitTask,
					taskName[i], configMINIMAL_STACK_SIZE,
					(void *) i, DEFAULT_THREAD_PRIO, &xLegGait[i] );
	}

	status = xTaskCreate( hexapodWalkingTask,
				 ( const char * ) "Walking",
				 configMINIMAL_STACK_SIZE,
				 NULL,
				 DEFAULT_THREAD_PRIO + 1,
				 &xWalkingTask );

	if(status != pdPASS){
		xil_printf("Can not create Walking task.\r\n");
	}

	xil_printf("Initialed\r\n");
	vTaskDelete(NULL);
	for(;;);
}

#endif //else if GEN_TEST_APP == 0

