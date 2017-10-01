/*
 * config.h
 *
 *  Created on: May 7, 2016
 *      Author: compi
 */

#pragma once
#include <limits.h>

#define USE_I2C
#define USE_SERIAL
//#define DEBUG_SERIAL
#define USE_LCD
//#define USE_LEDBAR

//#define DEBUG_TIMERS
#define DEBUG_DETECTOR
//#define DEBUG_SERIALIN

#define SHIFT_BELOW		4
#define SHIFT_BASE		0
#define SHIFT_ABOVE		2
#define SHIFT_ACTIVE	SCHAR_MIN
#define SHIFT_TIMEOUT	6

#define SHIFT_SUM		14
#define	DIVIDER			800
#define MCCOUNT			8
#define	SHIFT_TOLERANCE	2

#define TIMELIMIT		180	//	seconds

#define LCDADDRESS	(0x27 << 1)
#define LEDBAR1ADDRESS (0x20 << 1)
#define LEDBAR2ADDRESS (0x21 << 1)
#define EEPROMADDR 	0xA0
#define EESTART		0

