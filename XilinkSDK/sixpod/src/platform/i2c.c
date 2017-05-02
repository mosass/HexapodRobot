/*
 * i2c.c
 *
 *  Created on: Feb 14, 2017
 *      Author: Phanomphon Yotchon
 */

#include "i2c.h"
#include "sleep.h"
#include "platform.h"

XIicPs I2cInts;


int initI2c(void){
	int status;
	XIicPs_Config *I2cConfig;

	if(I2cInts.IsReady){
		return XST_SUCCESS;
	}

	I2cConfig = XIicPs_LookupConfig(XPAR_PS7_I2C_0_DEVICE_ID);
	if(I2cConfig == NULL){
		return XST_FAILURE;
	}

	status = XIicPs_CfgInitialize(&I2cInts, I2cConfig, I2cConfig->BaseAddress);
	if(status != XST_SUCCESS){
		return XST_FAILURE;
	}

	status = XIicPs_SelfTest(&I2cInts);
	if (status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	XIicPs_SetSClk(&I2cInts, IIC_SCLK_RATE);

	return XST_SUCCESS;
}

int i2cWrite(u8 slaveAddr, u8 regAddr, u8 length, u8 *data){
	int status;
#if USE_FREERTOS == 1
	taskENTER_CRITICAL();
#endif
	status = initI2c();
	if (status != XST_SUCCESS) {
#if USE_FREERTOS == 1
		taskEXIT_CRITICAL();
#endif
		return XST_FAILURE;
	}

	u8 dataSend[length + 1];
	dataSend[0] = regAddr;
	for(int i = 1; i < length + 1; i++){
		dataSend[i] = data[i-1];
	}

	status = XIicPs_MasterSendPolled(&I2cInts, dataSend, length + 1,
			slaveAddr);
	if (status != XST_SUCCESS) {
#if USE_FREERTOS == 1
		taskEXIT_CRITICAL();
#endif
		return XST_FAILURE;
	}

	while (XIicPs_BusIsBusy(&I2cInts)) {
		/* NOP */
	}
	usleep(500);
#if USE_FREERTOS == 1
	taskEXIT_CRITICAL();
#endif
	return XST_SUCCESS;
}

int i2cRead(u8 slaveAddr, u8 regAddr, u8 length, u8 *data){
	int status;
#if USE_FREERTOS == 1
	taskENTER_CRITICAL();
#endif
	status = initI2c();
	if (status != XST_SUCCESS) {
#if USE_FREERTOS == 1
		taskEXIT_CRITICAL();
#endif
		return XST_FAILURE;
	}

	status = XIicPs_MasterSendPolled(&I2cInts, &regAddr, 1,
			slaveAddr);
	if (status != XST_SUCCESS) {
#if USE_FREERTOS == 1
		taskEXIT_CRITICAL();
#endif
		return XST_FAILURE;
	}

	while (XIicPs_BusIsBusy(&I2cInts)) {
		/* NOP */
	}

	status = XIicPs_MasterRecvPolled(&I2cInts, data, length,
			slaveAddr);
	if (status != XST_SUCCESS) {
#if USE_FREERTOS == 1
		taskEXIT_CRITICAL();
#endif
		return XST_FAILURE;
	}
	usleep(500);
#if USE_FREERTOS == 1
	taskEXIT_CRITICAL();
#endif
	return XST_SUCCESS;
}


