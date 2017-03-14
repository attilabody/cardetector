/*
 * i2clcd.c
 *
 *  Created on: Mar 5, 2017
 *      Author: compi
 */
#include <stm32plus/i2clcd.h>
#include "stm32plus/strutil.h"

// commands
#define LCD_CLEARDISPLAY 0x01
#define LCD_RETURNHOME 0x02
#define LCD_ENTRYMODESET 0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_CURSORSHIFT 0x10
#define LCD_FUNCTIONSET 0x20
#define LCD_SETCGRAMADDR 0x40
#define LCD_SETDDRAMADDR 0x80

// flags for display entry mode
#define LCD_ENTRYRIGHT 0x00
#define LCD_ENTRYLEFT 0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00

// flags for display on/off control
#define LCD_DISPLAYON 0x04
#define LCD_DISPLAYOFF 0x00
#define LCD_CURSORON 0x02
#define LCD_CURSOROFF 0x00
#define LCD_BLINKON 0x01
#define LCD_BLINKOFF 0x00

// flags for display/cursor shift
#define LCD_DISPLAYMOVE 0x08
#define LCD_CURSORMOVE 0x00
#define LCD_MOVERIGHT 0x04
#define LCD_MOVELEFT 0x00

// flags for function set
#define LCD_8BITMODE 0x10
#define LCD_4BITMODE 0x00
#define LCD_2LINE 0x08
#define LCD_1LINE 0x00
#define LCD_5x10DOTS 0x04
#define LCD_5x8DOTS 0x00

// flags for backlight control
#define LCD_BACKLIGHT 0x08
#define LCD_NOBACKLIGHT 0x00

#define En 0b00000100  // Enable bit
#define Rw 0b00000010  // Read/Write bit
#define Rs 0b00000001  // Register select bit

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
static const uint8_t g_rowOffsets[4]  =  { 0x00, 0x40, 0x14, 0x54 };
static const uint8_t g_initNibbles[4] = {
		0x30, 0x30, 0x30, 0x20
};

static const uint8_t g_initBytes[5] = {
		0x28,
		0x01,	//clear display
		0x06,	//increment mode, no display shift
		0x0C,	//display on, hide cursor, not blinking
};


//////////////////////////////////////////////////////////////////////////////
//
//////////////////////////////////////////////////////////////////////////////
void I2cLcd_Init(I2cLcd_State *st, I2cMaster_State *i2cst, uint16_t i2cAddress)
{
	st->i2c = i2cst;
	st->data = LCD_BACKLIGHT;
	st->i2cAddress = i2cAddress;
}

//////////////////////////////////////////////////////////////////////////////
// 200 us @ 100kHz
inline HAL_StatusTypeDef I2cLcd_SendData(I2cLcd_State *st)
{
	return I2cMaster_Write(st->i2c, st->i2cAddress, &st->data, sizeof(st->data));
}

//////////////////////////////////////////////////////////////////////////////
// 400 us @ 100kHz
inline HAL_StatusTypeDef I2cLcd_Epulse(I2cLcd_State *st)
{
	HAL_StatusTypeDef	ret;
	st->data |= En;
	ret = I2cLcd_SendData(st);
	st->data &= ~En;
	if(ret == HAL_OK)
		return I2cLcd_SendData(st);
	return ret;
}

//////////////////////////////////////////////////////////////////////////////
// 600 us @ 100kHz
inline HAL_StatusTypeDef I2cLcd_SendNibble(I2cLcd_State *st, uint8_t nibble)
{
	HAL_StatusTypeDef ret;
	st->data = ((st->data & 0x0f) | (nibble & 0xf0));
	ret = I2cLcd_SendData(st);
	if(ret == HAL_OK)
		return I2cLcd_Epulse(st);
	return ret;
}

//////////////////////////////////////////////////////////////////////////////
inline HAL_StatusTypeDef I2cLcd_SendByte(I2cLcd_State *st, uint8_t b, uint8_t isCmd)
{
	HAL_StatusTypeDef ret;

	st->data = ((st->data & 0x0f & ~Rs) | (b & 0xf0) | (isCmd ? 0 : Rs));
	ret = I2cLcd_SendData(st);
	if(ret == HAL_OK) {
		ret = I2cLcd_Epulse(st);
		if(ret == HAL_OK) {
			st->data = ((st->data & 0x0f & ~Rs) | ((b & 0x0f) << 4) | (isCmd ? 0 : Rs));
			ret = I2cLcd_SendData(st);
			if(ret == HAL_OK)
				return I2cLcd_Epulse(st);
		}
	}
	return ret;
}

//////////////////////////////////////////////////////////////////////////////
HAL_StatusTypeDef I2cLcd_InitDisplay(I2cLcd_State *st )
{
	HAL_StatusTypeDef ret;
	uint8_t count;

	st->data = LCD_BACKLIGHT;
	if((ret = I2cLcd_SendData(st)) == HAL_OK)
	{
		HAL_Delay(500);
		for(count = 0; count < sizeof(g_initNibbles); count++){
			if((ret = I2cLcd_SendNibble(st, g_initNibbles[count])) != HAL_OK)
				return ret;
			HAL_Delay(5);
		}
		for (count = 0; count < sizeof(g_initBytes); count++) {
			if((ret = I2cLcd_SendByte(st, g_initBytes[count], 1)) != HAL_OK)
				return ret;
			HAL_Delay(3);
		}
	}
	return ret;
}

//////////////////////////////////////////////////////////////////////////////
HAL_StatusTypeDef I2cLcd_Clear(I2cLcd_State *st)
{
	HAL_StatusTypeDef ret = I2cLcd_SendByte(st, LCD_CLEARDISPLAY, 1);	// clear display, set cursor position to zero
	HAL_Delay(3);												// this command takes a long time!
	return ret;
}

//////////////////////////////////////////////////////////////////////////////
HAL_StatusTypeDef I2cLcd_Home(I2cLcd_State *st)
{
	HAL_StatusTypeDef ret = I2cLcd_SendByte(st, LCD_RETURNHOME, 1);		// set cursor position to zero
	HAL_Delay(2);												// this command takes a long time!
	return ret;
}

//////////////////////////////////////////////////////////////////////////////
HAL_StatusTypeDef I2cLcd_SetCursor(I2cLcd_State *st, uint8_t x, uint8_t y)
{
	return I2cLcd_SendByte(st, LCD_SETDDRAMADDR | (x + g_rowOffsets[y & 3]), 1);
}

//////////////////////////////////////////////////////////////////////////////
HAL_StatusTypeDef I2cLcd_PrintStr(I2cLcd_State *st, const char *str)
{
	HAL_StatusTypeDef ret = HAL_OK;
	while(*str) {
		ret = I2cLcd_SendByte(st, *str++, 0);
		if(ret != HAL_OK)
			return ret;
	}
	return ret;
}

//////////////////////////////////////////////////////////////////////////////
uint32_t I2cLcd_PrintChar(I2cLcd_State *st, const char c)
{
	return I2cLcd_SendByte(st, c, 0);
}

//////////////////////////////////////////////////////////////////////////////
size_t I2cLcd_PrintUint(I2cLcd_State *st, uint32_t u, uint8_t hex)
{
	char	buffer[11];
	size_t ret = hex ? uitohex(buffer, u) : uitodec(buffer, u);
	I2cLcd_PrintStr(st, buffer);
	return ret;
}

//////////////////////////////////////////////////////////////////////////////
size_t I2cLcd_PrintInt(I2cLcd_State *st, int32_t i, uint8_t hex)
{
	char	buffer[12];
	char	*tmp = buffer;
	size_t ret = 0;
	if(i < 0) {
		*tmp++ = '-';
		i = -i;
		ret = 1;
	}
	ret += hex ? uitohex(tmp, i) : uitodec(tmp, i);
	I2cLcd_PrintStr(st, buffer);
	return ret;
}

//#endif	//	#if defined(HAVE_I2C) && defined(USE_I2C)

