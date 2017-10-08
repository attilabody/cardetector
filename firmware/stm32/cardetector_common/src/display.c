#include <config.h>
#include <cardetector_common/liveconfig.h>
#include <cardetector_common/display.h>
#include <globals.h>

//////////////////////////////////////////////////////////////////////////////
extern LIVECONFIG	g_config;

//////////////////////////////////////////////////////////////////////////////
#if defined(USE_LCD)
I2cLcd_State	g_lcd;
#elif defined(USE_LEDBAR)
Pcf8574_Status	g_ledbars[2];
#endif


//////////////////////////////////////////////////////////////////////////////
void InitializeDisplay(I2cMaster_State *i2c)
{
#if defined(USE_LCD)
	I2cLcd_Init(&g_lcd, i2c, LCDADDRESS );
	I2cLcd_InitDisplay(&g_lcd);
#elif defined(USE_LEDBAR)
	Pcf8574_Init(&g_ledbars[0], i2c, LEDBAR1ADDRESS);
	Pcf8574_Init(&g_ledbars[1], i2c, LEDBAR2ADDRESS);
#endif
}

#if defined(USE_LCD) || defined(USE_LEDBAR)
////////////////////////////////////////////////////////////////////
uint8_t CalcBar(const volatile DETECTORSTATUS *channel)
{
	if(!channel->diff) return 0;

	uint8_t	neg = channel->diff < 0;
	int16_t	absdiff = !neg ? channel->diff : -channel->diff;
	int16_t	ret = 0;

	if(absdiff < channel->tolerance) return 0;

	absdiff -= channel->tolerance;
	int16_t	threshold = channel->threshold - channel->tolerance;

	ret = ((int32_t)absdiff * (DISPLAY_TOP - 1))/threshold + 1;
	if(ret > DISPLAY_LIMIT) ret = DISPLAY_LIMIT;
	return neg ? ret : -ret;
}

////////////////////////////////////////////////////////////////////
void UpdateBar(uint8_t line, int8_t chars)
{
#if defined(USE_LCD)
#if !defined(DEBUG_LCD)
	I2cLcd_SetCursor(&g_lcd, 0, line);
	int8_t pos;
	if (chars < 0) {
		pos = -16;
		while (pos < chars) {
			I2cLcd_PrintChar(&g_lcd, ' ');
			++pos;
		}
		while (chars++ < 0)
			I2cLcd_PrintChar(&g_lcd, '<');
	} else {
		pos = 0;
		while (chars--) {
			I2cLcd_PrintChar(&g_lcd, pos < DISPLAY_TOP ? '>' : '#');
			++pos;
		}
		while (pos++ < 16)
			I2cLcd_PrintChar(&g_lcd, ' ');
	}
#endif	//	! DEBUG_LCD
#elif defined(USE_LEDBAR)
	Pcf8574_WritePort(
			&g_ledbars[line],
			g_config.ledbarvalues[line][chars < 0 ? 0 : chars + 1]
		);
#endif	//	USE_LEDBAR
}
#endif	//	defined(USE_LCD) || defined(USE_LEDBAR)
