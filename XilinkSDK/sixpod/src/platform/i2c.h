/*
 * i2c.h
 *
 *  Created on: Feb 14, 2017
 *      Author: Phanomphon Yotchon
 */

#include "xparameters.h"
#include "xiicps.h"


#ifndef SRC_IMU_I2C_H_
#define SRC_IMU_I2C_H_

#define IIC_SCLK_RATE		100000


int initI2c(void);
int i2cWrite(u8 slaveAddr, u8 regAddr, u8 length, u8 *data);
int i2cRead(u8 slaveAddr, u8 regAddr, u8 length, u8 *data);

#endif /* SRC_IMU_I2C_H_ */
