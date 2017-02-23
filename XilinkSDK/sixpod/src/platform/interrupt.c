/*
 * interrupt.c
 *
 *  Created on: Feb 12, 2017
 *      Author: Phanomphon Yotchon
 */

#include "interrupt.h"

/************************** Variable Definitions *****************************/

/*
 * The following are declared globally so they are zeroed and so they are
 * easily accessible from a debugger
 */
XGpioPs GpioInst; /* The Instance of the GPIO Driver */
XScuGic GicInst; /* The Instance of the Interrupt Controller Driver */

int setupIntrSystem(int pinNumber, void (*callback)(XGpioPs*, u32, u32), int intrType){
	int status;
	if(callback == NULL){
		return XST_FAILURE;
	}

	status = initScuGic();
	if(status != XST_SUCCESS){
		return XST_FAILURE;
	}

	Xil_ExceptionInit();

	Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_IRQ_INT,
			(Xil_ExceptionHandler)XScuGic_InterruptHandler, &GicInst);

	XScuGic_Connect(&GicInst, XPAR_PS7_GPIO_0_INTR,
			(Xil_ExceptionHandler)XGpioPs_IntrHandler, (void *) &GpioInst);

	status = initIntrGpio(pinNumber, callback, intrType);
	if(status != XST_SUCCESS){
		return XST_FAILURE;
	}

	XScuGic_Enable(&GicInst, XPAR_PS7_GPIO_0_INTR);

	Xil_ExceptionEnable();

	return XST_SUCCESS;
}

int initScuGic(void){
	int status;
	XScuGic_Config *IntcConfig;

	IntcConfig = XScuGic_LookupConfig(XPAR_PS7_SCUGIC_0_DEVICE_ID);
	if(IntcConfig == NULL){
		return XST_FAILURE;
	}
	status = XScuGic_CfgInitialize(&GicInst, IntcConfig, IntcConfig->CpuBaseAddress);
	if(status != XST_SUCCESS){
		return XST_FAILURE;
	}

	return XST_SUCCESS;
}

// intrType can be XGPIOPS_IRQ_TYPE_*
int initIntrGpio(int pinNumber, void (*callback)(XGpioPs*, u32, u32), int intrType){
	XGpioPs_Config *gpioConf;
	int status;
	gpioConf = XGpioPs_LookupConfig(XPAR_PS7_GPIO_0_DEVICE_ID);
	if(gpioConf == NULL){
		return XST_FAILURE;
	}
	XGpioPs_CfgInitialize(&GpioInst, gpioConf, gpioConf->BaseAddr);
	/* Run a self-test on the GPIO device. */
	status = XGpioPs_SelfTest(&GpioInst);
	if (status != XST_SUCCESS) {
		return XST_FAILURE;
	}
	XGpioPs_SetDirectionPin(&GpioInst, pinNumber, 0); // input port

	XGpioPs_SetIntrTypePin(&GpioInst, pinNumber, intrType);
//	XGpioPs_IntrEnablePin(&GpioInst, pinNumber);
	XGpioPs_SetCallbackHandler(&GpioInst, (void *) &GpioInst, (XGpioPs_Handler)callback);
	XGpioPs_IntrClearPin(&GpioInst, pinNumber);

	return XST_SUCCESS;
}

void enableIntr(int pinNumber){
	XGpioPs_IntrEnablePin(&GpioInst, pinNumber);
	return;
}

void disableIntr(int pinNumber){
	XGpioPs_IntrDisablePin(&GpioInst, pinNumber);
	return;
}

