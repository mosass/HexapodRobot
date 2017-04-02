/*
 * Main C++ Application
 */

#define GEN_TEST_APP  1

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
#include "xstatus.h"
/* Module includes */
#include "hexapod.h"

#if GEN_TEST_APP == 1
#include "testApp.h"
int main (void) {
	xil_printf("Test\r\n");
	testIMU();
	return XST_SUCCESS;
}
#elif GEN_TEST_APP == 0

int main (void) {
	xil_printf("Test\r\n");
//	hexapodInitial();
	xil_printf("Ok\r\n");

	for(;;){
//		hexapodReadIMU();
	}
//	testIMU();
	return XST_SUCCESS;
}
#endif //else if GEN_TEST_APP == 0

