/*
 * ioexttest.c
 *
 *  Created on: Jun 19, 2016
 *      Author: compi
 */
#include "config.h"
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>

#include <stdlib.h>
#include <limits.h>

#include "serial.h"
#include "i2c.h"
#include "pcf8574.h"
#include "i2c_lcd.h"

#if defined(IOEXTTEST)

PCF8574_STATUS g_lcdps;

volatile uint32_t	g_ms = 0;

////////////////////////////////////////////////////////////////////
uint32_t	millis()
{
	cli();
	uint32_t	tmp = g_ms;
	sei();
	return tmp;
}

////////////////////////////////////////////////////////////////////
ISR (TIMER2_COMPA_vect)
{
	++g_ms;
}

////////////////////////////////////////////////////////////////////
void display(char *string)
{
	int count = 0;

	while(*string && count++ < 16)
		lcd_sendbyte(*string++, 1);
}

////////////////////////////////////////////////////////////////////
int main(void)
{
	uint8_t			data = 0x55;
	uint32_t		prevms = 0;
	PCF8574_STATUS	st;

	sei();
	i2c_begin();

	TCCR2A = 0;						//t2: stop
	TCCR2B = 0;
	GTCCR = _BV(PSRASY);			//t2: prescaler reset
	TCNT2 = 0;						//t2: reset counter

	TCCR2A = _BV(WGM21);			//t2: CTC mode
	OCR2A = 124;					//t2: 62.5*128*125 = 1000000 ns = 1 ms
	TIMSK2 = _BV(OCIE2A);			//t2 enable Timer2 Interrupt

	TCCR2B = _BV(CS20) | _BV(CS22);	//t2: prescaler 128 (start)

//	pcf8574_init(&st, 0x20, 0xff);

	_delay_ms(500);  //Initiaize LCD
	lcd_init();
	_delay_ms(200);
	lcd_clear();
	lcd_setcursor(1, 0);
	display("Hi");
	display(" There....");
	lcd_setcursor(0, 1);
	display("Circuits4You.com");


	while(1)
	{
//		pcf8574_write8(&st, data);
//		pcf8574_read8(&st, &data);
//		data ^= 0xff;
//		while(millis() - prevms < 1000 );
//		prevms += 1000;
	}
}
#endif	//	IOEXTTEST
