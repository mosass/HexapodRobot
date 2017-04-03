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
	testLegModule();
	return XST_SUCCESS;
}
#elif GEN_TEST_APP == 0

int main (void) {
	xil_printf("Test\r\n");
	Link3d L1(5, 5, 5);
	Link3d L2(10, 20, 30);

	Link3d L3 = -L2 + 6;
	xil_printf("%d, %d, %d\r\n", (int)L1.a, (int)L1.b, (int)L1.c);
	xil_printf("%d, %d, %d\r\n", (int)L2.a, (int)L2.b, (int)L2.c);
	xil_printf("%d, %d, %d\r\n", (int)L3.a, (int)L3.b, (int)L3.c);

	return XST_SUCCESS;
}
#endif //else if GEN_TEST_APP == 0

