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

int main (void) {
	while(1){
		xil_printf("Test\r\n");
		sleep(1);
	}


	return XST_SUCCESS;
}
#endif //else if GEN_TEST_APP == 0

