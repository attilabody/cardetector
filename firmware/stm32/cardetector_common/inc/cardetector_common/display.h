/*
 * display.h
 *
 *  Created on: Mar 15, 2017
 *      Author: compi
 */

#pragma once
#include <config.h>
#include <cardetector_common/detector.h>
#include <stm32plus/i2clcd.h>
#include <stm32plus/pcf8574.h>

#if defined(USE_LCD)
#	define DISPLAY_TOP 13
#	define DISPLAY_LIMIT 16
#else
#	define DISPLAY_TOP 6
#	define DISPLAY_LIMIT 7
#endif

#if defined(USE_LCD)
extern I2cLcd_State		g_lcd;
#endif

#if defined(USE_LEDBAR)
extern Pcf8574_Status	g_ledbars[2];
#endif

void InitializeDisplay(I2cMaster_State *i2c);
uint8_t CalcBar(const volatile DETECTORSTATUS *channel);
void UpdateBar(uint8_t line, int8_t chars);
