/*
 * timer2test.c
 *
 *  Created on: May 7, 2016
 *      Author: compi
 */
#include "config.h"
#include <avr/io.h>
#include <avr/interrupt.h>

#if defined(TIMER2TEST)

volatile uint16_t tel;

ISR(TIMER2_COMPA_vect)
{
	tel = (tel + 1);
	if (tel >= 1250) {
		tel = 0; PORTB ^= _BV(PORTB5);
	}
}

int main( void )
{
	DDRB   |= (1<<DDB5); //Set the 6th bit on PORTB (i.e. PB5) to 1 => output

	TCNT2   = 0;
	TCCR2A |= _BV(WGM21); // Configure timer 2 for CTC mode
	TCCR2B |= _BV(CS22); // Start timer at Fcpu/64
	TIMSK2 |= _BV(OCIE2A); // Enable CTC interrupt
	OCR2A   = 250; // Set CTC compare value with a prescaler of 64

	sei(); // Enable global interrupts

	while ( 1 ) { ;; }
	return 0;
}
#endif	//	TIMER2TEST
