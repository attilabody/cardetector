/*
 * globals.h
 *
 *  Created on: Mar 15, 2017
 *      Author: compi
 */
#pragma once
#include <stm32plus/i2ceeprom.h>
#include <stm32plus/i2clcd.h>
#include <stm32plus/pcf8574.h>
#include <config.h>

extern I2cMaster_State	*g_i2c;
#if defined(USE_LCD)
extern I2cLcd_State		g_lcd;
#endif
#if defined(USE_LEDBAR)
extern Pcf8574_Status	g_ledbars[2];
#endif
extern I2cEEPROM_State	g_eeprom;

extern uint8_t			g_lineBuffer[64];
extern volatile uint8_t	g_lineReceived;

extern uint8_t			g_debug;
