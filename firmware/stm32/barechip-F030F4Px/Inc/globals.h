
#pragma once
#include <stm32plus/i2ceeprom.h>
#include <stm32plus/i2clcd.h>
#include <stm32plus/pcf8574.h>
#include <cardetector_common/liveconfig.h>
#include <cardetector_common/detector.h>
#include <config.h>

#if defined(USE_I2C)
extern I2cMaster_State	*g_i2c;
#if defined(USE_LCD)

extern I2cLcd_State		g_lcd;

#elif defined(USE_LEDBAR)

extern Pcf8574_Status	g_ledbars[2];

#endif

#if defined(USE_EEPROM)
extern I2cEEPROM_State	g_eeprom;
#endif
#endif	//	I2C

#if defined(USE_SERIAL)
extern uint8_t			g_lineBuffer[64];
extern volatile uint8_t	g_lineReceived;
#endif

extern LIVECONFIG 		g_config;
extern DETECTORSTATUS	g_statuses[2];
