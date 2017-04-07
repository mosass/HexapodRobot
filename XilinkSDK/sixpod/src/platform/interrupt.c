/*
 * interrupt.c
 *
 *  Created on: Feb 12, 2017
 *      Author: Phanomphon Yotchon
 */

#include "interrupt.h"

#define USE_FREERTOS	1
/************************** Variable Definitions *****************************/

/*
 * The following are declared globally so they are zeroed and so they are
 * easily accessible from a debugger
 */
XGpioPs GpioInst; /* The Instance of the GPIO Driver */
XScuGic* GicInst; /* The Instance of the Interrupt Controller Driver */

#if USE_FREERTOS == 0
XScuGic XScuGicInst;
#endif

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
			(Xil_ExceptionHandler)XScuGic_InterruptHandler, GicInst);

	Xil_AssertNonvoid(GicInst != NULL);
	Xil_AssertNonvoid(XPAR_PS7_GPIO_0_INTR < XSCUGIC_MAX_NUM_INTR_INPUTS);
	Xil_AssertNonvoid(XGpioPs_IntrHandler != NULL);
	Xil_AssertNonvoid(GicInst->IsReady == XIL_COMPONENT_IS_READY);

	XScuGic_Connect(GicInst, XPAR_PS7_GPIO_0_INTR,
			(Xil_ExceptionHandler)XGpioPs_IntrHandler, (void *) &GpioInst);

	status = initIntrGpio(pinNumber, callback, intrType);
	if(status != XST_SUCCESS){
		return XST_FAILURE;
	}

	XScuGic_Enable(GicInst, XPAR_PS7_GPIO_0_INTR);

	Xil_ExceptionEnable();

	return XST_SUCCESS;
}

int initScuGic(void){
	int status;
#if USE_FREERTOS == 1
	GicInst = (XScuGic*) prvGetInterruptControllerInstance();
#else
	GicInst = &XScuGicInst;
#endif
	XScuGic_Config *IntcConfig;

	IntcConfig = XScuGic_LookupConfig(XPAR_PS7_SCUGIC_0_DEVICE_ID);
	if(IntcConfig == NULL){
		return XST_FAILURE;
	}

	status = XScuGic_CfgInitialize(GicInst, IntcConfig, IntcConfig->CpuBaseAddress);
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

