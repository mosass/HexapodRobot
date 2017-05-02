/*
 * comm.c
 *
 *  Created on: Feb 5, 2017
 *      Author: Phanomphon Yotchon
 */

#include "comm.h"
#include "platform.h"

XUartPs Uart_Ps;

/*
 * Initializes Uart_Ps instance.
 *
 * @return
 * 		- XST_SUCCESS if initialization was successful or Uart_Ps is already initial.
 * 		- XST_FAILURE if initialization was failure.
 */
int CommPortInitial() {
	// return success if Uart_Ps is already initial.
	if (Uart_Ps.IsReady == XIL_COMPONENT_IS_READY) {
		return XST_SUCCESS;
	}

	int Status;
	XUartPs_Config *Config;

	// Look up the configuration of Uart_Ps
	Config = XUartPs_LookupConfig(UART_DEVICE_ID);
	if (NULL == Config) {
		return XST_FAILURE;
	}

	// Initialize configuration for Uart_Ps
	Status = XUartPs_CfgInitialize(&Uart_Ps, Config, Config->BaseAddress);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	// Set Uart_Ps baud rate.
	Status = XUartPs_SetBaudRate(&Uart_Ps, UART_BAUDRATE);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	// Initialize successful.
	return XST_SUCCESS;
}

/*
 * This function sends the data buffer using the Uart_Ps in polled mode.
 *
 * @param	DataPtr is a pointer to a buffer of data to be sent.
 * @param	NumBytes is number of bytes to be sent.
 *
 * @return
 * 		- XST_SUCCESS if sending was complete.
 * 		- XST_FALIURE if number of bytes sent does not match with NumBytes parameter.
 */
int CommPortSend(u8 *DataPtr, u32 NumBytes) {
#if USE_FREERTOS == 1
	taskENTER_CRITICAL();
#endif
	unsigned int send_count = XUartPs_Send(&Uart_Ps, DataPtr, NumBytes);

	if (send_count != NumBytes) {
#if USE_FREERTOS == 1
		taskEXIT_CRITICAL();
#endif
		return XST_FAILURE;
	}

	// wait for sending complete.
	while (XUartPs_IsSending(&Uart_Ps))
		;

#if USE_FREERTOS == 1
	taskEXIT_CRITICAL();
#endif
	return XST_SUCCESS;
}

/*
 * This function receive a specified number of bytes of data
 * from Uart_Ps. This function is blocking, it wait for data coming until count of bytes
 * is equal number request bytes.
 *
 * @param	BufferPtr is a pointer to a buffer for data to be received into.
 * @param	NumBytes is number of bytes to be received.
 *
 * @return
 * 		- XST_SUCCESS if number of bytes received does match with
 * 		NumBytes parameter.
 */
int CommPortRecv(u8 *BufferPtr, u32 NumBytes) {
#if USE_FREERTOS == 1
	taskENTER_CRITICAL();
#endif
	unsigned int recv_count = 0;
	//u32 baseAddr = Uart_Ps.Config.BaseAddress;
	while (recv_count < NumBytes) {
		BufferPtr[recv_count] = XUartPs_RecvByte(Uart_Ps.Config.BaseAddress);
		recv_count += 1;
	}

#if USE_FREERTOS == 1
	taskEXIT_CRITICAL();
#endif
	return XST_SUCCESS;
}

/*
 * This function send write word command to joint controller to controls dynamixel servo.
 *
 * @param	Id contain the number id of dynamixel servo.
 * @param	Address is register address to write word data into.
 * @param	Value is word data to write into specified address.
 *
 * @return
 * 		- XST_SUCCESS if sending command is successful.
 * 		- XST_FAILURE if error was occur.
 */
int CommPortWriteWord(u8 Id, u8 Address, u16 Value) {
	int status;
	u8 cmd[WRITE_WORD_CMD_LENGTH];

	//parse write word command
	cmd[0] = WRITE_WORD_OPCODE | (Id & 0x1F);
	cmd[1] = Address;
	cmd[2] = (Value >> 8) & 0xFF;
	cmd[3] = Value & 0xFF;
	cmd[4] = ~(cmd[0] + cmd[1] + cmd[2] + cmd[3]);

	status = CommPortInitial();
	if (XST_FAILURE == status) {
		return XST_FAILURE;
	}

	//send command
	status = CommPortSend(cmd, WRITE_WORD_CMD_LENGTH);
	if (XST_FAILURE == status) {
		return XST_FAILURE;
	}

	//wait for ACK signal
	u8 res;
	status = CommPortRecv(&res, 1);
	if (XST_FAILURE == status) {
		return XST_FAILURE;
	}

	//check response
	if (res == ACK) {
		return XST_SUCCESS;
	}

	return XST_FAILURE;
}

/*
 * This function send write byte command to joint controller to controls dynamixel servo.
 *
 * @param	Id contain the number id of dynamixel servo.
 * @param	Address is register address to write byte data into.
 * @param	Value is byte data to write into specified address.
 *
 * @return
 * 		- XST_SUCCESS if sending command is successful.
 * 		- XST_FAILURE if error was occur.
 */
int CommPortWriteByte(u8 Id, u8 Address, u8 Value){
	int status;
	u8 cmd[WRITE_BYTE_CMD_LENGTH];

	//parse write byte command
	cmd[0] = WRITE_BYTE_OPCODE | (Id & 0x1F);
	cmd[1] = Address;
	cmd[2] = Value & 0xFF;
	cmd[3] = ~(cmd[0] + cmd[1] + cmd[2]);

	status = CommPortInitial();
	if(XST_FAILURE == status){
		return XST_FAILURE;
	}

	//send command
	status = CommPortSend(cmd, WRITE_BYTE_CMD_LENGTH);
	if(XST_FAILURE == status){
		return XST_FAILURE;
	}

	//wait for ACK signal
	u8 res;
	status = CommPortRecv(&res, 1);
	if(XST_FAILURE == status){
		return XST_FAILURE;
	}

	//check response
	if(res == ACK){
		return XST_SUCCESS;
	}

	return XST_FAILURE;
}

/*
 * This function send read word command to joint controller to read data from dynamixel servo.
 *
 * @param	Id contain the number id of dynamixel servo.
 * @param	Address is register address to read word data.
 *
 * @return
 * 		- The data if reading successfully.
 * 		- XST_FAILURE if error was occur.
 */
int CommPortReadWord(u8 Id, u8 Address) {
	int status;
	u8 cmd[READ_CMD_LENGTH];

	//parse read word command
	cmd[0] = READ_WORD_OPCODE | (Id & 0x1F);
	cmd[1] = Address;
	cmd[2] = ~(cmd[0] + cmd[1]);

	status = CommPortInitial();
	if (XST_FAILURE == status) {
		return XST_FAILURE;
	}

	//send command
	status = CommPortSend(cmd, READ_CMD_LENGTH);
	if (XST_FAILURE == status) {
		return XST_FAILURE;
	}

	//wait for ACK signal
	u8 res;
	status = CommPortRecv(&res, 1);
	if (XST_FAILURE == status) {
		return XST_FAILURE;
	}

	//check response
	if (res == ACK) {

		//read two byte data
		u8 recv_data[2];
		status = CommPortRecv(recv_data, 2);

		if (XST_SUCCESS == status) {
			u16 res_data = (recv_data[0] << 8) | recv_data[1];
			return res_data;
		}
	}

	return XST_FAILURE;
}

/*
 * This function send read word command to joint controller to read data from dynamixel servo.
 *
 * @param	Id contain the number id of dynamixel servo.
 * @param	Address is register address to read byte data.
 *
 * @return
 * 		- The data if reading successfully.
 * 		- XST_FAILURE if error was occur.
 */
int CommPortReadByte(u8 Id, u8 Address) {
	int status;
	u8 cmd[READ_CMD_LENGTH];

	//parse read byte command
	cmd[0] = READ_BYTE_OPCODE | (Id & 0x1F);
	cmd[1] = Address;
	cmd[2] = ~(cmd[0] + cmd[1]);

	status = CommPortInitial();
	if (XST_FAILURE == status) {
		return XST_FAILURE;
	}

	//send command
	status = CommPortSend(cmd, READ_CMD_LENGTH);
	if (XST_FAILURE == status) {
		return XST_FAILURE;
	}

	//wait for ACK signal
	u8 res;
	status = CommPortRecv(&res, 1);
	if (XST_FAILURE == status) {
		return XST_FAILURE;
	}

	//check response
	if (res == ACK) {

		//read one byte data
		u8 recv_data;

		status = CommPortRecv(&recv_data, 1);
		if (XST_SUCCESS == status) {
			return recv_data;
		}
	}
	return XST_FAILURE;
}
