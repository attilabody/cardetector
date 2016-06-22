/*
 * i2c_lcd.h
 *
 *  Created on: Jun 20, 2016
 *      Author: compi
 */
#if !defined(I2C_LCD_H_) && defined(HAVE_I2C) && defined(USE_I2C)
#define I2C_LCD_H_

#include "pcf8574.h"

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

void 	i2clcd_init(PCF8574_STATUS *ps, unsigned char i2caddress);
void 	i2clcd_sendbyte(PCF8574_STATUS *ps, unsigned char b, unsigned char mode);
void 	i2clcd_epulse(PCF8574_STATUS *ps);
void 	i2clcd_clear(PCF8574_STATUS *ps);
void 	i2clcd_home(PCF8574_STATUS *ps);
void 	i2clcd_setcursor(PCF8574_STATUS *ps, uint8_t col, uint8_t row);
void	i2clcd_print(PCF8574_STATUS *ps, const char *string);
int		i2clcd_printlong(PCF8574_STATUS *ps, long l);
inline void	i2clcd_printchar(PCF8574_STATUS *ps, char c) { i2clcd_sendbyte(ps, c, 1); }

#endif /* I2C_LCD_H_ */
