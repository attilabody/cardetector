/*
 * setup.c
 *
 *  Created on: Jul 3, 2016
 *      Author: compi
 */
#include "config.h"
#include <avr/io.h>

#if defined(__AVR_ATmega328P__)
////////////////////////////////////////////////////////////////////
void setup()
{
	PORTB = 0xff & ~_BV(DDB_OUT) & ~_BV(DDB_LED1) & ~_BV(DDB_LED2);
	PORTC = PORTD = 0xff;	//activate pullups on all inputs
	DDRB = _BV(DDB_OUT) | _BV(DDB_LED1) | _BV(DDB_LED2);
	DDRC = _BV(DDB_LED5) | _BV(DDB_LED6) | _BV(DDB_LED7) | _BV(DDB_LED8);
	DDRD = _BV(DDB_LED3) | _BV(DDB_LED4);

	TCCR0A = 0;						//t0: stop
	TCCR0B = 0;
	TCNT0 = 0;						//t0: reset counter

	TIMSK0 = _BV(TOIE0);			//t0: overflow interrupt enable

	TCCR2A = 0;						//t2: stop
	TCCR2B = 0;
	GTCCR = _BV(PSRASY);			//t2: prescaler reset
	TCNT2 = 0;						//t2: reset counter

	TCCR2A = _BV(WGM21);			//t2: CTC mode
	OCR2A = 124;					//t2: 62.5*128*125 = 1000000 ns = 1 ms
	TIMSK2 = _BV(OCIE2A);			//t2 enable Timer2 Interrupt

	TCCR2B = _BV(CS20) | _BV(CS22);	//t2: prescaler 128 (start)

	TCCR0B = _BV(CS00) | _BV(CS01) | _BV(CS02);	//t0: ext clk rising edge (start)
}  // end of setup

#elif defined(__AVR_ATtiny85__)
////////////////////////////////////////////////////////////////////
void setup()
{
	PORTB = 0xff & ~_BV(BIT_LED1) & ~_BV(BIT_OUT);
	DDRB |= _BV(DD_LED1) | _BV(DDB_OUT);

	TCCR0A = 0;						//t0: stop
	TCCR0B = 0;
	TCNT0 = 0;						//t0: reset counter

	TCCR1 = 0;						//t1: stop
	GTCCR |= _BV(PSR1);				//t1: prescaler reset
	TCNT1 = 0;						//t1: reset counter
	OCR1C = 124;					//for counter autoreset
	OCR1A = 124;					//for interrupt

	TIMSK = _BV(TOIE0) | _BV(OCIE1A);	//enable interrupts for t0 & t1

	TCCR1 |= _BV(CS13) | _BV(CTC1);	//t1: prescaler 128, CTC mode (start)
	TCCR0B = _BV(CS00) | _BV(CS01) | _BV(CS02);	//t0: ext clk rising edge (start)

}  // end of setup
#endif

