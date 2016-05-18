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
volatile uint8_t	g_counterReady = 0;
volatile uint8_t	g_overrun = 0;
volatile uint32_t	g_ms = 0;

// internal to counting routine
volatile uint8_t g_overflowCount = 0;
volatile uint16_t g_timerTicks = 0;
uint16_t g_timerPeriod = 250;

void setup(uint8_t ms);

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
	, 6		//limithistf
	, 60	//tlimit
};




const PARAMS g_params =
{
	  {5, 3, 0, 8}	//below, above, active, timeout
	, 8		//shift
	, 6		//limithistf
	, 60	//tlimit
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
#endif	//	#if (defined(DEBUG_TIMERS) || defined(DEBUG_DETECTOR)) && defined(HAVE_SERIAL)

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
#endif	//	#if (defined(DEBUG_TIMERS) || defined(DEBUG_DETECTOR)) && defined(HAVE_SERIAL)
	debugmod = (diff << g_params.shifts[state]);

	g_sum += debugmod;

#if defined(DEBUG_DETECTOR) && defined(HAVE_SERIAL)
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

#if (defined(DEBUG_TIMERS) || defined(DEBUG_DETECTOR)) && defined(HAVE_SERIAL)
	uart_init();
#endif	//	#if (defined(DEBUG_TIMERS) || defined(DEBUG_DETECTOR)) && defined(HAVE_SERIAL)

	DDRB |= _BV(DD_LED) | _BV(DD_OUT);
	PORTB |= _BV(PORT_LED);
	PORTB &= ~_BV(PORT_OUT);

	sei();

	setup(250);  // ms

#if (defined(DEBUG_TIMERS) || defined(DEBUG_DETECTOR)) && defined(HAVE_SERIAL)
	uart_print("Calibrating: ");
#endif	//	#if (defined(DEBUG_TIMERS) || defined(DEBUG_DETECTOR)) && defined(HAVE_SERIAL)

	//calibration
	for(count = 0; count < 4; ++count)
	{
		while(!g_counterReady);
		g_counterReady = 0;
		PORTB ^= _BV(PORT_LED);
#if (defined(DEBUG_TIMERS) || defined(DEBUG_DETECTOR)) && defined(HAVE_SERIAL)
		uart_transmit('x');
#endif
	}

	for(count = 0; count < 16; ++count)
	{
		while(!g_counterReady);
		g_sum += g_timerCounts;
		g_counterReady = 0;
		PORTB ^= _BV(PORT_LED);
#if (defined(DEBUG_TIMERS) || defined(DEBUG_DETECTOR)) && defined(HAVE_SERIAL)
		uart_transmit('.');
#endif	//	#if (defined(DEBUG_TIMERS) || defined(DEBUG_DETECTOR)) && defined(HAVE_SERIAL)
	}
	g_sum <<= (g_params.shift - 4);
#if (defined(DEBUG_TIMERS) || defined(DEBUG_DETECTOR)) && defined(HAVE_SERIAL)
	uart_println("");
#endif	//	#if (defined(DEBUG_TIMERS) || defined(DEBUG_DETECTOR)) && defined(HAVE_SERIAL)

	PORTB &= ~_BV(PORT_LED);

#if (defined(DEBUG_TIMERS) || defined(DEBUG_DETECTOR)) && defined(HAVE_SERIAL)
	uart_print("Sum: ");
	uart_printlong(g_sum);
	uart_println("");
#endif	//	#if (defined(DEBUG_TIMERS) || defined(DEBUG_DETECTOR)) && defined(HAVE_SERIAL)

	while(1)
	{
		while (!g_counterReady);

		uint32_t	countercopy = g_timerCounts;
		g_counterReady = 0;

#if defined(DEBUG_DETECTOR) && defined(HAVE_SERIAL)
		float frq = (countercopy * 1000.0) / g_timerPeriod;
		unsigned long lf = (unsigned long)frq;

		uart_print("Freq: ");
		uart_printlong(lf);
#endif	//	DEBUG_DETECTOR

		if(detect(countercopy)) {
			if(!prevactive) {
				prevactive = 1;
				PORTB |= (_BV(PORT_LED) | _BV(PORT_OUT));
			}
		} else {
			if(prevactive) {
				prevactive = 0;
				PORTB &= ~(_BV(PORT_LED) | _BV(PORT_OUT));
			}
		}

	}
}

////////////////////////////////////////////////////////////////////
void setup(uint8_t ms)
{
	g_timerPeriod = ms;             // how many 1 ms counts to do

#if defined(__AVR_ATtiny85__)
	TCCR0A = 0;						//t0: stop
	TCCR0B = 0;
	TCNT0 = 0;						//t0: reset counter

	TCCR1 = 0;						//t1: stop
	GTCCR |= _BV(PSR1);				//t1: prescaler reset
	TCNT1 = 0;						//t1: reset counter
	OCR1C = 124;					//for counter autoreset
	OCR1A = 124;					//for interrupt

	TIMSK = _BV(TOIE0) | _BV(OCIE1A);	//enable interrupts for t0 & t1

	TCCR0B = _BV(CS00) | _BV(CS01) | _BV(CS02);	//t0: ext clk rising edge (start)
	TCCR1 |= _BV(CS13) | _BV(CTC1);	//t1: prescaler 128, CTC mode (start)
#elif defined(__AVR_ATmega328P__)
	TCCR0A = 0;				//t0: stop
	TCCR0B = 0;
	TCNT0 = 0;				//t0: reset counter

	TIMSK0 = _BV(TOIE0);	//t0: overflow interrupt enable

	TCCR2A = 0;				//t2: stop
	TCCR2B = 0;
	GTCCR = _BV(PSRASY);	//t2: prescaler reset
	TCNT2 = 0;				//t2: reset counter

	TCCR2A = _BV(WGM21);	//t2: CTC mode
	OCR2A = 124;			//t2: 62.5*128*125 = 1000000 ns = 1 ms
	TIMSK2 = _BV(OCIE2A);	//t2 enable Timer2 Interrupt

	TCCR2B = _BV (CS20) | _BV(CS22);				//t2: prescaler 128 (start)
	TCCR0B = _BV (CS00) | _BV(CS01) | _BV(CS02);	//t0: ext clk, rising edge(start)
#else
#error "Only ATmega 328P and ATtinyX5 are supported."
#endif

}  // end of setup

////////////////////////////////////////////////////////////////////
ISR(COUNTERVECT)
{
	++g_overflowCount;               // count number of Counter1 overflows
}  // end of TIMER1_OVF_vect

////////////////////////////////////////////////////////////////////
//  Timer Interrupt Service is invoked every 1 ms = 1000 Hz
//  16Mhz / 128 / 125 = 1000 Hz
ISR (TIMERVECT)
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
	if ((TIFR & _BV(TOV0)) && counterValue < 128)
		overflowCopy++;

	// end of gate time, measurement ready
	// calculate total count
	g_timerCounts = (overflowCopy << 8) + counterValue; // each overflow is 256 more
	g_counterReady = 1;              // set global flag for end count period
}  // end of TIMER2_COMPA_vect

#if defined(DEBUG_TIMERS) && defined(HAVE_SERIAL)
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
