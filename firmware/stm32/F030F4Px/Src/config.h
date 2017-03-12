/*
 * config.h
 *
 *  Created on: May 7, 2016
 *      Author: compi
 */

#ifndef CONFIG_H_
#define CONFIG_H_

#include <limits.h>

//#define F_CPU 16000000UL
//#define BAUD 115200

#define USE_I2C
#define USE_SERIAL

//#define DEBUG_TIMERS
#define DEBUG_DETECTOR
//#define DEBUG_SERIALIN

#define SHIFT_BELOW		5
#define SHIFT_BASE		0
#define SHIFT_ABOVE		3
#define SHIFT_ACTIVE	SCHAR_MIN
#define SHIFT_TIMEOUT	7

#define SHIFT_SUM		16
#define	DIVIDER			800
#define MCCOUNT			8
#define	SHIFT_TOLERANCE	2

#define TIMELIMIT			180	//	seconds

#define LCD_I2C_ADDRESS	0x27

#endif /* CONFIG_H_ */