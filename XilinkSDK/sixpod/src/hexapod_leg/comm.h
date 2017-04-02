/*
 * comm.h
 *
 *  Created on: Feb 5, 2017
 *      Author: Phanomphon Yotchon
 */

#include "xuartps.h"

#ifndef SRC_LEGS_CONTROLS_COMM_H_
#define SRC_LEGS_CONTROLS_COMM_H_

/************************** Constant Definitions *****************************/

/*
 *
 */
#define UART_DEVICE_ID              XPAR_XUARTPS_0_DEVICE_ID
#define UART_BAUDRATE				115200

#define WRITE_WORD_OPCODE 			0xE0
#define WRITE_BYTE_OPCODE			0xC0
#define READ_WORD_OPCODE 			0xA0
#define READ_BYTE_OPCODE 			0x80

#define WRITE_WORD_CMD_LENGTH		5
#define WRITE_BYTE_CMD_LENGTH		4
#define READ_CMD_LENGTH				3

#define ACK							0xAA
#define NACK						0x55

/************************** Function Prototypes ******************************/
int CommPortInitial();
int CommPortRecv(u8 *BufferPtr, u32 NumBytes);
int CommPortSend(u8 *DataPtr, u32 NumBytes);

int CommPortWriteWord(u8 Id, u8 Address, u16 Value);
int CommPortWriteByte(u8 Id, u8 Address, u8 Value);
int CommPortReadWord(u8 Id, u8 Address);
int CommPortReadByte(u8 Id, u8 Address);


#endif /* SRC_LEGS_CONTROLS_COMM_H_ */
