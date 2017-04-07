/*
 * interrupt.h
 *
 *  Created on: Feb 12, 2017
 *      Author: Phanomphon Yotchon
 */

#include "FreeRTOS.h"
#include "xparameters.h"
#include "xgpiops.h"
#include "xil_exception.h"
#include "xscugic.h"

#ifndef SRC_IMU_INTERRUPT_H_
#define SRC_IMU_INTERRUPT_H_

/************************** Constant Definitions *****************************/
/*
 * The following constants map to the XPAR parameters created in the
 * xparameters.h file. They are defined here such that a user can easily
 * change all the needed parameters in one place.
 */
#define INTR_TYPE_EDGE_RISING 		XGPIOPS_IRQ_TYPE_EDGE_RISING
#define INTR_TYPE_EDGE_FALLING 		XGPIOPS_IRQ_TYPE_EDGE_FALLING
#define INTR_TYPE_EDGE_BOTH 		XGPIOPS_IRQ_TYPE_EDGE_BOTH
#define INTR_TYPE_LEVEL_HIGH 		XGPIOPS_IRQ_TYPE_LEVEL_HIGH
#define INTR_TYPE_LEVEL_LOW 		XGPIOPS_IRQ_TYPE_LEVEL_LOW


/************************** Function Prototypes ******************************/
int setupIntrSystem(int pinNumber, void (*callback)(XGpioPs*, u32, u32), int intrType);
int initScuGic(void);
int initIntrGpio(int pinNumber, void (*callback)(XGpioPs*, u32, u32), int intrType);
void enableIntr(int pinNumber);
void disableIntr(int pinNumber);

#endif /* SRC_IMU_INTERRUPT_H_ */
