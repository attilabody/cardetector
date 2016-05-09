/*
 * cardetector.c
 *
 *  Created on: Apr 29, 2016
 *      Author: compi
 */
#include "config.h"
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include <stdlib.h>

#include "serial.h"

volatile unsigned long g_timerCounts;
volatile unsigned char g_counterReady;

// internal to counting routine
volatile unsigned long g_overflowCount;
volatile unsigned int g_timerTicks;
unsigned int g_timerPeriod;

void startCounting(unsigned int ms);

#ifdef DUMPINFO
void dumpinfo();
#endif	//	DUMPINFO

//******************************************************************
int main(void)
{
	uart_init();

	DDRB |= (1<<DDB5); //Set the 6th bit on PORTB (i.e. PB5) to 1 => output

	sei();

	while(1)
	{
		startCounting(200);  // how many ms to count for

		while (!g_counterReady) {
		}  // loop until count over

		// adjust counts by counting interval to give frequency in Hz
		float frq = (g_timerCounts * 1000.0) / g_timerPeriod;

		unsigned long lf = (unsigned long)frq;

		uart_print("Frequency: ");
		uart_printlong(lf);
		uart_print(" Hz. Raw counter value:");
		uart_printlong(g_timerCounts);
		uart_println("");
	}
}

//******************************************************************
void startCounting(unsigned int ms)
{
	g_counterReady = 0;         // time not up yet
	g_timerPeriod = ms;             // how many 1 ms counts to do
	g_timerTicks = 0;               // reset interrupt counter
	g_overflowCount = 0;            // no overflows yet

	// reset Timer 0 and Timer 2
	TCCR0A = 0;
	TCCR0B = 0;
	TCCR2A = 0;
	TCCR2B = 0;

	// Timer 0 - counts events on pin D4
	TIMSK0 = _BV(TOIE0);   // interrupt on Timer 0 overflow

	// Timer 2 - gives us our 1 ms counting interval
	// 16 MHz clock (62.5 ns per tick) - prescaled by 128
	//  counter increments every 8 µs.
	// So we count 125 of them, giving exactly 1000 µs (1 ms)
	TCCR2A = _BV(WGM21);   // CTC mode
	OCR2A = 124;            // count up to 125  (zero relative!!!!)

	// Timer 2 - interrupt on match (ie. every 1 ms)
	TIMSK2 = _BV(OCIE2A);   // enable Timer2 Interrupt

	TCNT0 = 0;      // Both counters to zero
	TCNT2 = 0;

	// Reset prescalers
	GTCCR = _BV(PSRASY);        // reset prescaler now
	// start Timer 2
	TCCR2B = _BV (CS20) | _BV(CS22);  // prescaler of 128
	// start Timer 0
	// External clock source on T0 pin (D4). Clock on rising edge.
	TCCR0B = _BV (CS00) | _BV(CS01) | _BV(CS02);
}  // end of startCounting

//******************************************************************
ISR(TIMER0_OVF_vect)
{
	++g_overflowCount;               // count number of Counter1 overflows
}  // end of TIMER1_OVF_vect

//******************************************************************
//  Timer2 Interrupt Service is invoked by hardware Timer 2 every 1 ms = 1000 Hz
//  16Mhz / 128 / 125 = 1000 Hz

ISR (TIMER2_COMPA_vect)
{
	// grab counter value before it changes any more
	unsigned char counterValue = TCNT0;
	unsigned long overflowCopy = g_overflowCount;

	// see if we have reached timing period
	if (++g_timerTicks < g_timerPeriod)
		return;  // not yet

	// if just missed an overflow
	if ((TIFR0 & _BV(TOV0)) && counterValue < 128)
		overflowCopy++;

	// end of gate time, measurement ready

	TCCR0A = 0;    // stop timer 0
	TCCR0B = 0;

	TCCR2A = 0;    // stop timer 2
	TCCR2B = 0;

	TIMSK0 = 0;    // disable Timer0 Interrupt
	TIMSK2 = 0;    // disable Timer2 Interrupt

	// calculate total count
	g_timerCounts = (overflowCopy << 8) + counterValue; // each overflow is 256 more
	g_counterReady = 1;              // set global flag for end count period
}  // end of TIMER2_COMPA_vect

#ifdef DUMPINFO
//******************************************************************
void dumpreg16(char *name, unsigned int value )
{
	char			buffer[sizeof(value)*8+1];
	int				i;
	unsigned int	tmp = value;

	for(i=0; i<sizeof(tmp)*8; ++i) {
		buffer[i] = (tmp & 0x8000)? '1' : '0';
		tmp <<= 1;
	}
	buffer[i] = 0;

	uart_print(name);
	uart_print(": ");
	uart_print(buffer);
	uart_print(" (0x");
	itoa(value, buffer, 16);
	uart_print(buffer);
	uart_println(")");
}

//******************************************************************
void dumpreg(char *name, unsigned char value )
{
	char			buffer[sizeof(value)*8+1];
	int				i;
	unsigned char	tmp = value;

	for(i=0; i<sizeof(tmp)*8; ++i) {
		buffer[i] = (tmp & 0x80)? '1' : '0';
		tmp <<= 1;
	}
	buffer[i] = 0;

	uart_print(name);
	uart_print(": ");
	uart_print(buffer);
	uart_print(" (0x");
	itoa(value, buffer, 16);
	uart_print(buffer);
	uart_println(")");
}

//******************************************************************
void dumpvar32(char *name, unsigned long value)
{
	char buffer[sizeof(value)*8+1];

	uart_print(name);
	uart_print(": 0x");
	itoa(value, buffer, 16);
	uart_print(buffer);
	uart_println("");
}

//******************************************************************
void dumpinfo()
{
	uart_println("\n-----------------------------------------------------");
	dumpreg("DDRD", DDRD);
	dumpreg("PORTD", PORTD);
	dumpreg("PIND", PIND);
	uart_println("");
	dumpreg("TCCR1A", TCCR1A);
	dumpreg("TCCR1B", TCCR1B);
	dumpreg("TCCR1C", TCCR1C);
	dumpreg("TIMSK1", TIMSK1);
	dumpreg("TIFR1", TIFR1);
	dumpreg16("TCNT1", TCNT1);
	dumpreg16("OCR1A", OCR1A);
	dumpreg16("ICR1", ICR1);
	uart_println("");
	dumpvar32("g_overflowCount", g_overflowCount);
	dumpvar32("g_timerCounts", g_timerCounts);
	uart_println("");
//	dumpreg("", );
//	dumpreg("", );
//	dumpreg("", );
}
#endif	//	DUMPINFO
