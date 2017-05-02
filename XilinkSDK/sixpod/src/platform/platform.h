/*
 * config.h
 *
 *  Created on: May 2, 2017
 *      Author: mosas
 */

#ifndef SRC_PLATFORM_PLATFORM_H_
#define SRC_PLATFORM_PLATFORM_H_

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
#include "netif/xadapter.h"
#include "lwip/sockets.h"
#include "lwipopts.h"

/* Config */
#define USE_FREERTOS 	1

#endif /* SRC_PLATFORM_PLATFORM_H_ */
