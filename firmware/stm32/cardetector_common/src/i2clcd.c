/*
 * i2clcd.c
 *
 *  Created on: Mar 5, 2017
 *      Author: compi
 */
#include <cardetector_common/i2clcd.h>
#include "cardetector_common/strutil.h"

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
static const uint8_t m_rowOffsets[4]  =  { 0x00, 0x40, 0x14, 0x54 };
static const uint8_t m_init[5] = {
		0x28,
		0x01,	//clear display
		0x06,	//increment mode, no display shift
		0x0C,	//display on, hide cursor, not blinking
};


//////////////////////////////////////////////////////////////////////////////
//
//////////////////////////////////////////////////////////////////////////////
void I2cLcd_Init(I2cLcd_Status *st, I2cMaster_Status *i2cst, uint16_t i2cAddress)
//: m_i2c(i2c)
//, st->m_data(LCD_BACKLIGHT)
//, m_i2cAddress(i2cAddress)
{
	st->m_i2c = i2cst;
	st->m_data = LCD_BACKLIGHT;
	st->m_i2cAddress = i2cAddress;
}

//////////////////////////////////////////////////////////////////////////////
// 200 us @ 100kHz
inline HAL_StatusTypeDef I2cLcd_SendData(I2cLcd_Status *st)
{
	return I2cMaster_Write(st->m_i2c, st->m_i2cAddress, &st->m_data, sizeof(st->m_data), It);
}

//////////////////////////////////////////////////////////////////////////////
// 400 us @ 100kHz
inline HAL_StatusTypeDef I2cLcd_Epulse(I2cLcd_Status *st)
{
	HAL_StatusTypeDef	ret;
	st->m_data |= En;
	ret = I2cLcd_SendData(st);
	st->m_data &= ~En;
	if(ret == HAL_OK)
		return I2cLcd_SendData(st);
	return ret;
}

//////////////////////////////////////////////////////////////////////////////
// 600 us @ 100kHz
inline HAL_StatusTypeDef I2cLcd_SendNibble(I2cLcd_Status *st, uint8_t nibble)
{
	HAL_StatusTypeDef ret;
	st->m_data = ((st->m_data & 0x0f) | (nibble & 0xf0));
	ret = I2cLcd_SendData(st);
	if(ret == HAL_OK)
		return I2cLcd_Epulse(st);
	return ret;
}

//////////////////////////////////////////////////////////////////////////////
inline HAL_StatusTypeDef I2cLcd_SendByte(I2cLcd_Status *st, uint8_t b, uint8_t isCmd)
{
	HAL_StatusTypeDef ret;

	st->m_data = ((st->m_data & 0x0f & ~Rs) | (b & 0xf0) | (isCmd ? 0 : Rs));
	ret = I2cLcd_SendData(st);
	if(ret == HAL_OK) {
		ret = I2cLcd_Epulse(st);
		if(ret == HAL_OK) {
			st->m_data = ((st->m_data & 0x0f & ~Rs) | ((b & 0x0f) << 4) | (isCmd ? 0 : Rs));
			ret = I2cLcd_SendData(st);
			if(ret == HAL_OK)
				return I2cLcd_Epulse(st);
		}
	}
	return ret;
}

//////////////////////////////////////////////////////////////////////////////
HAL_StatusTypeDef I2cLcd_InitDisplay(I2cLcd_Status *st )
{
	HAL_StatusTypeDef ret;
	uint8_t count;

	st->m_data = LCD_BACKLIGHT;
	ret = I2cLcd_SendData(st);
	if(ret == HAL_OK) {
		HAL_Delay(500);
		ret = I2cLcd_SendNibble(st, 0x30);
		if(ret == HAL_OK) {
			HAL_Delay(5);
			ret = I2cLcd_SendNibble(st, 0x30);
			if(ret == HAL_OK) {
				ret = I2cLcd_SendNibble(st, 0x30);
				if(ret == HAL_OK) {
					ret = I2cLcd_SendNibble(st, 0x20);
					if(ret == HAL_OK)
						for (count = 0; count < sizeof(m_init); count++) {
							ret = I2cLcd_SendByte(st, m_init[count], 1);
							if(ret != HAL_OK)
								return ret;
							HAL_Delay(3);
						}
	}}}}
	return ret;
}

//////////////////////////////////////////////////////////////////////////////
HAL_StatusTypeDef I2cLcd_Clear(I2cLcd_Status *st)
{
	HAL_StatusTypeDef ret = I2cLcd_SendByte(st, LCD_CLEARDISPLAY, 1);	// clear display, set cursor position to zero
	HAL_Delay(3);												// this command takes a long time!
	return ret;
}

//////////////////////////////////////////////////////////////////////////////
HAL_StatusTypeDef I2cLcd_Home(I2cLcd_Status *st)
{
	HAL_StatusTypeDef ret = I2cLcd_SendByte(st, LCD_RETURNHOME, 1);		// set cursor position to zero
	HAL_Delay(2);												// this command takes a long time!
	return ret;
}

//////////////////////////////////////////////////////////////////////////////
HAL_StatusTypeDef I2cLcd_SetCursor(I2cLcd_Status *st, uint8_t x, uint8_t y)
{
	return I2cLcd_SendByte(st, LCD_SETDDRAMADDR | (x + m_rowOffsets[y & 3]), 1);
}

//////////////////////////////////////////////////////////////////////////////
HAL_StatusTypeDef I2cLcd_PrintStr(I2cLcd_Status *st, const char *str)
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
size_t I2cLcd_PrintInt(I2cLcd_Status *st, uint32_t u, uint8_t hex)
{
	char	buffer[11];
	size_t ret = hex ? uitohex(buffer, u) : uitodec(buffer, u);
	I2cLcd_PrintStr(st, buffer);
	return ret;
}


//#endif	//	#if defined(HAVE_I2C) && defined(USE_I2C)

