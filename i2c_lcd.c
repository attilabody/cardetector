/*
 * i2c_lcd.c
 *
 *  Created on: Jun 20, 2016
 *      Author: compi
 */

#include "config.h"
#if defined(HAVE_I2C) && defined(USE_I2C)
#include <stdlib.h>
#include <string.h>
#include <util/delay.h>

#include "i2c_lcd.h"
#include "pcf8574.h"
#include "config.h"


static void lcd_sendnibble(PCF8574_STATUS *ps, unsigned char nibble)
{
	pcf8574_write8(ps, (ps->data & 0x0f) | (nibble & 0xf0));
	i2clcd_epulse(ps);
}

//=================================================================
//
//=================================================================

//=================================================================
//        LCD Display Initialization Function
//=================================================================
void i2clcd_init(PCF8574_STATUS *ps, unsigned char i2caddress)
{
	pcf8574_init(ps, i2caddress, LCD_BACKLIGHT);
	pcf8574_write_st(ps);

	_delay_ms(500);
	int count;
	static const unsigned char init[]={0x01,0x0C,0x06,0x02,0x02};

	lcd_sendnibble(ps, 0x30);
	_delay_us(4500);
	lcd_sendnibble(ps, 0x30);
	_delay_us(4500);
	lcd_sendnibble(ps, 0x30);
	_delay_us(150);
	lcd_sendnibble(ps, 0x20);

	for (count = 0; count < sizeof(init); count++) {
		i2clcd_sendbyte(ps, init[count], 0);
		_delay_us(4500);
	}
}

//=================================================================
//        Enable Pulse Function
//=================================================================
void i2clcd_epulse(PCF8574_STATUS *ps)
{
	ps->data |= En;
	pcf8574_write_st(ps);
	_delay_us(1); //Adjust delay if required
	ps->data &= ~En;
	pcf8574_write_st(ps);
	_delay_us(50); //Adjust delay if required
}

//=================================================================
//        Send Single Byte to LCD Display Function
//=================================================================
void i2clcd_sendbyte(PCF8574_STATUS *ps, unsigned char b, unsigned char mode)
{
	pcf8574_write8(ps, (ps->data & 0x0f & ~Rs) | (b & 0xf0) | (mode ? Rs : 0));
	i2clcd_epulse(ps);
	pcf8574_write8(ps, (ps->data & 0x0f & ~Rs) | (b << 4) | (mode ? Rs : 0));
	i2clcd_epulse(ps);
}

void i2clcd_clear(PCF8574_STATUS *ps)
{
	i2clcd_sendbyte(ps, LCD_CLEARDISPLAY, 0);// clear display, set cursor position to zero
	_delay_ms(2);  // this command takes a long time!
}

void i2clcd_home(PCF8574_STATUS *ps)
{
	i2clcd_sendbyte(ps, LCD_RETURNHOME, 0);  // set cursor position to zero
	_delay_ms(2);  // this command takes a long time!
}

void i2clcd_setcursor(PCF8574_STATUS *ps, uint8_t col, uint8_t row)
{
	static const int row_offsets[] = { 0x00, 0x40, 0x14, 0x54 };
	i2clcd_sendbyte(ps, LCD_SETDDRAMADDR | (col + row_offsets[row]), 0);
}

void i2clcd_print(PCF8574_STATUS *ps, const char *string)
{
	while(*string) i2clcd_sendbyte(ps, *string++, 1);
}

int i2clcd_printlong(PCF8574_STATUS *ps, long l)
{
	char	buffer[sizeof(l)*3+1];
	ltoa(l, buffer, 10);
	i2clcd_print(ps, buffer);
	return strlen(buffer);
}

#endif	//	#if defined(HAVE_I2C) && defined(USE_I2C)
