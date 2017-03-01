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
#include "imu.h"

#include "testApp.h"

int main (void) {
	testIMU();
	return XST_SUCCESS;
}
