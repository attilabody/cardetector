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
#include <avr/eeprom.h>

#include <stdlib.h>

#include "serial.h"

volatile uint16_t	g_timerCounts;
volatile uint8_t	g_counterReady;
volatile uint8_t	g_overrun;
volatile uint32_t	g_ms = 0;

// internal to counting routine
volatile uint8_t g_overflowCount;
volatile uint16_t g_timerTicks;
uint16_t g_timerPeriod;

void startCounting(uint8_t ms);

#ifdef DEBUG_TIMERS
void dumpinfo();
#endif	//	DUMPINFO

enum STATES
{
	BELOW,
	ABOVE,
	ACTIVE,
	TOUT
};

typedef struct
{
	uint8_t	shifts[4];
	uint8_t	shift, limitshift, tlimit;
} PARAMS;

const PARAMS EEMEM ee_params =
{
	  {5, 3, 0, 8}	//below, above, active, timeout
	, 8		//shift
	, 5		//limithistf
	, 40	//tlimit
};




const PARAMS g_params =
{
	  {5, 3, 0, 8}	//below, above, active, timeout
	, 8		//shift
	, 5		//limithistf
	, 40	//tlimit
};

uint32_t		g_sum = 0;


////////////////////////////////////////////////////////////////////
void init_config()
{
	eeprom_read_block((void*)&g_params, &ee_params, sizeof(g_params));
}

////////////////////////////////////////////////////////////////////
uint8_t	detect(uint16_t counter)
{
	static uint16_t			time=0;

	uint16_t		debugavg;
#if defined(DEBUG_DETECTOR)
	uint16_t		debugth;
#endif	//	#if defined(DEBUG_TIMERS) || defined(DEBUG_DETECTOR)

	int32_t			debugmod;

	enum STATES		state;
	int32_t			diff;

	diff = (int32_t)counter - (int32_t)(debugavg = (g_sum >> g_params.shift));		// adjust counts by counting interval to give frequency in Hz

	if (diff < 0)
		state = BELOW;
	else if(diff < (g_sum >> (g_params.shift + g_params.limitshift)))
		state = ABOVE;
	else if (time < ((uint16_t)g_params.tlimit) << 2)
		state = ACTIVE;
	else
		state = TOUT;

#if defined(DEBUG_DETECTOR)
	debugth = (g_sum >> (g_params.shift + g_params.limitshift));
#endif	//	#if defined(DEBUG_TIMERS) || defined(DEBUG_DETECTOR)
	debugmod = (diff << g_params.shifts[state]);

	g_sum += debugmod;

#ifdef DEBUG_DETECTOR
	uart_print(" Sum: ");
	uart_printlong(g_sum);
	uart_print(" Raw: ");
	uart_printlong(g_timerCounts);
	uart_print(" Avg: ");
	uart_printlong(debugavg);
	uart_print(" Diff: ");
	uart_printlong(diff);
	uart_print(" Sta: ");
	uart_printlong(state);
	uart_print(" << : ");
	uart_printlong(g_params.shifts[state]);
	uart_print(" Mod: ");
	uart_printlong(debugmod);
	uart_print(" Th: ");
	uart_printlong(debugth);
	uart_println("");
#endif	//	DEBUG_DETECTOR

	if(state < ACTIVE) {
		time = 0;
		return 0;
	} else {
		++time;
		return 1;
	}


}

////////////////////////////////////////////////////////////////////
int main(void)
{
	uint8_t			prevactive = 0;
	uint8_t			count;

#if defined(DEBUG_TIMERS) || defined(DEBUG_DETECTOR)
	uart_init();
#endif	//	#if defined(DEBUG_TIMERS) || defined(DEBUG_DETECTOR)

	DDRB |= (1<<DDB5); //Set the 6th bit on PORTB (i.e. PB5) to 1 => output
	PORTB |= (1 << PORTB5);

	sei();

	startCounting(250);  // how many ms to count for

#if defined(DEBUG_TIMERS) || defined(DEBUG_DETECTOR)
	uart_print("Calibrating: ");
#endif	//	#if defined(DEBUG_TIMERS) || defined(DEBUG_DETECTOR)

	//calibration
	for(count = 0; count < 4; ++count)
	{
		while(!g_counterReady);
		g_counterReady = 0;
		PORTB ^= (1 << PORTB5);
#if defined(DEBUG_TIMERS) || defined(DEBUG_DETECTOR)
		uart_transmit('x');
#endif
	}

	for(count = 0; count < 16; ++count)
	{
		while(!g_counterReady);
		g_sum += g_timerCounts;
		g_counterReady = 0;
		PORTB ^= (1 << PORTB5);
#if defined(DEBUG_TIMERS) || defined(DEBUG_DETECTOR)
		uart_transmit('.');
#endif	//	#if defined(DEBUG_TIMERS) || defined(DEBUG_DETECTOR)
	}
	g_sum <<= (g_params.shift - 4);
#if defined(DEBUG_TIMERS) || defined(DEBUG_DETECTOR)
	uart_println("");
#endif	//	#if defined(DEBUG_TIMERS) || defined(DEBUG_DETECTOR)

	PORTB &= ~(1<<PORTB5);

#if defined(DEBUG_TIMERS) || defined(DEBUG_DETECTOR)
	uart_print("Sum: ");
	uart_printlong(g_sum);
	uart_println("");
#endif	//	#if defined(DEBUG_TIMERS) || defined(DEBUG_DETECTOR)

	while(1)
	{
		while (!g_counterReady) {
		}  // loop until count over

		uint32_t	countercopy = g_timerCounts;
		g_counterReady = 0;

#ifdef DEBUG_DETECTOR
		float frq = (countercopy * 1000.0) / g_timerPeriod;
		unsigned long lf = (unsigned long)frq;

		uart_print("Freq: ");
		uart_printlong(lf);
#endif	//	DEBUG_DETECTOR

		if(detect(countercopy)) {
			if(!prevactive) {
				prevactive = 1;
				PORTB |= (1<<PORTB5);
			}
		} else {
			if(prevactive) {
				prevactive = 0;
				PORTB &= ~(1<<PORTB5);
			}
		}

	}
}

////////////////////////////////////////////////////////////////////
void startCounting(uint8_t ms)
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

////////////////////////////////////////////////////////////////////
ISR(TIMER0_OVF_vect)
{
	++g_overflowCount;               // count number of Counter1 overflows
}  // end of TIMER1_OVF_vect

////////////////////////////////////////////////////////////////////
//  Timer2 Interrupt Service is invoked by hardware Timer 2 every 1 ms = 1000 Hz
//  16Mhz / 128 / 125 = 1000 Hz
ISR (TIMER2_COMPA_vect)
{
	// grab counter value before it changes any more
	uint8_t	counterValue = TCNT0;
	uint8_t	overflowCopy = g_overflowCount;

	++g_ms;
	// see if we have reached timing period
	if (++g_timerTicks < g_timerPeriod)
		return;  // not yet

	TCNT0 = 0;
	g_timerTicks = 0;
	g_overflowCount = 0;

	if(g_counterReady) {
		g_overrun = 1;
		return;
	}

	// if just missed an overflow
	if ((TIFR0 & _BV(TOV0)) && counterValue < 128)
		overflowCopy++;

	// end of gate time, measurement ready
	// calculate total count
	g_timerCounts = (overflowCopy << 8) + counterValue; // each overflow is 256 more
	g_counterReady = 1;              // set global flag for end count period
}  // end of TIMER2_COMPA_vect

#ifdef DEBUG_TIMERS
////////////////////////////////////////////////////////////////////
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

////////////////////////////////////////////////////////////////////
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

////////////////////////////////////////////////////////////////////
void dumpvar32(char *name, unsigned long value)
{
	char buffer[sizeof(value)*8+1];

	uart_print(name);
	uart_print(": 0x");
	itoa(value, buffer, 16);
	uart_print(buffer);
	uart_println("");
}

////////////////////////////////////////////////////////////////////
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
