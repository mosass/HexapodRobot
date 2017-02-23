/*
 * ArduinoWrapper.h
 *
 *  Created on: Mar 27, 2013
 *      Author: zoellner
 */

#ifndef ARDUINOWRAPPER_H_
#define ARDUINOWRAPPER_H_



//Standard Libraries
#include <stdint.h>
#include "xtime_l.h"

#define COUNTS_PER_MSECOND 		(XPAR_CPU_CORTEXA9_CORE_CLOCK_FREQ_HZ / (2U*1000U))

//TODO functions that need wrapper: millis(), Serial.print

uint32_t millis()
{
	XTime tCur;
	uint32_t count;
	XTime_GetTime(&tCur);
	count = tCur / COUNTS_PER_MSECOND;
	return count;
}

#endif /* ARDUINOWRAPPER_H_ */
