/*
 * cardetector.cpp
 *
 *  Created on: Apr 29, 2016
 *      Author: compi
 */
#include "config.h"
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>

#include <stdlib.h>
#include <limits.h>

#include "serial.h"
#include "i2c.h"
#include "pcf8574.h"
#include "i2c_lcd.h"
#include "utils.h"
#include "commsyms.h"

#if defined(DEBUG_DETECTOR)
volatile uint16_t	g_badirq = 0;
uint16_t			g_badirqcnt = 0;
#endif

volatile uint16_t	g_timer_counts;
volatile uint8_t	g_counter_ready = 0;
volatile uint8_t	g_overrun = 0;
volatile uint32_t	g_ms = 0;
#if defined(DEBUG_DETECTOR)
volatile uint32_t	g_lastwtf = 0;
#endif

volatile uint8_t	g_counter_overflows = 0;
volatile uint16_t	g_timer_overflows = 0;
uint16_t 			g_counting_period = 250;
uint16_t			g_active_time = 0;

#if defined(HAVE_SERIAL)
unsigned char 	g_linebuffer[64];
unsigned char	g_lineidx;
#endif

#if defined(HAVE_I2C) && defined(USE_I2C)
PCF8574_STATUS	g_ps;
#endif	//	defined(HAVE_I2C) && defined(USE_I2C)

void setup(uint8_t ms);
void processinput();

#ifdef DEBUG_TIMERS
void dumpinfo();
#endif	//	DUMPINFO

enum STATES
{
	BELOW = 0,
	BASE,
	ABOVE,
	ACTIVE,
	TOUT,
	STATECOUNT
};

typedef struct
{
	int8_t		shifts[5];
	uint8_t		sumshift, tlimit;
	uint16_t	divider;
} CONFIG;

CONFIG EEMEM ee_config =
{
	  {SHIFT_BELOW, SHIFT_BASE, SHIFT_ABOVE, SHIFT_ACTIVE, SHIFT_TIMEOUT}	//below, above, active, timeout
	, SHIFT_SUM		//shift
	, TIMELIMIT		//tlimit
	, DIVIDER		//divider
};

CONFIG 		g_config;
uint32_t	g_sum = 0;


////////////////////////////////////////////////////////////////////
uint32_t	millis()
{
	uint32_t	tmp;
	cli();
	tmp = g_ms;
	sei();
	return tmp;
}

////////////////////////////////////////////////////////////////////
void setleds(char red, char green, char blue)
{
	if(red|green|blue) PORTB |= _BV(PORT_LED1);
	else PORTB &= ~ _BV(PORT_LED1);
//	if(green) PORTB |= _BV(PORT_LED2);
//	else PORTB &= ~ _BV(PORT_LED2);
//	if(blue) PORTB |= _BV(PORT_LED3);
//	else PORTB &= ~ _BV(PORT_LED3);
}

////////////////////////////////////////////////////////////////////
void init_config()
{
	eeprom_read_block((void*)&g_config, &ee_config, sizeof(g_config));
}

////////////////////////////////////////////////////////////////////
enum STATES detect(uint16_t counter)
{
#if defined(DEBUG_DETECTOR) && defined(HAVE_SERIAL)
	uint16_t		debugavg = g_sum >> g_config.sumshift;
	int32_t			modifier;
#endif
	uint16_t		threshold = (g_sum >> g_config.sumshift) / g_config.divider;
	int8_t			shift;

	enum STATES		state;
	int32_t			diff = (int32_t)counter - (int32_t)(g_sum >> g_config.sumshift);
	uint16_t		tolerance = threshold >> SHIFT_TOLERANCE;


	if (diff < 0)
		state = tolerance + diff < 0 ? BELOW : BASE;
	else if( diff < tolerance)
		state = BASE;
	else if(diff < threshold)
		state = ABOVE;
	else if (g_active_time < ((uint16_t)g_config.tlimit) << 2)
		state = ACTIVE;
	else
		state = TOUT;

	shift = g_config.shifts[state];
	if(shift != SCHAR_MIN)
		g_sum += (shift >= 0) ? (diff << shift) : (diff >> -shift);

#if defined(DEBUG_DETECTOR) && defined(HAVE_SERIAL)
	modifier = (sumshift >= 0) ? (diff << sumshift) : (diff >> -sumshift);
	uart_printstr(" Sum: ");
	uart_printlong(g_sum);
	uart_printstr(" Raw: ");
	uart_printlong(g_timer_counts);
	uart_printstr(" Avg: ");
	uart_printlong(debugavg);
	uart_printstr(" Diff: ");
	uart_printlong(diff);
	uart_printstr(" Sta: ");
	uart_printlong(state);
	uart_printstr(" << : ");
	uart_printlong(g_config.shifts[state]);
	uart_printstr(" Mod: ");
	uart_printlong(modifier);
	uart_printstr(" Th: ");
	uart_printlong(threshold);
	uart_printstr(" To: ");
	uart_printlong(tolerance);
	uart_printstr(" BAD: ");
	uart_printlong(g_badirq);

	uart_println("");
#endif	//	DEBUG_DETECTOR

	if(state < ACTIVE) g_active_time = 0;
	else ++g_active_time;

	return state;
}

////////////////////////////////////////////////////////////////////
int main(void)
{
	uint8_t			count;
	enum STATES		state;

	DDRB |= _BV(DD_LED1) | _BV(DD_LED2) | _BV(DD_LED3) | _BV(DD_OUT);
	PORTB = 0;

	setup(250);  // ms
	sei();

#if defined(HAVE_SERIAL)
	uart_printstr("Calibrating: ");
#endif	//	#if (defined(DEBUG_TIMERS) || defined(DEBUG_DETECTOR)) && defined(HAVE_SERIAL)

	//calibration
	for(count = 0; count < 4; ++count)
	{
		while(!g_counter_ready);
		g_counter_ready = 0;
		if(count & 1) setleds(0, 0, 0);
		else setleds(1,0,0);
#if defined(HAVE_SERIAL)
		uart_printchar('x');
#endif
	}

	for(count = 0; count < 16; ++count)
	{
		while(!g_counter_ready);
		g_sum += g_timer_counts;
		g_counter_ready = 0;
		if(count & 1) setleds(0, 0, 0);
		else setleds(1,1,0);
#if defined(HAVE_SERIAL)
		uart_printchar('.');
#endif	//	#if (defined(DEBUG_TIMERS) || defined(DEBUG_DETECTOR)) && defined(HAVE_SERIAL)
	}
	g_sum <<= (g_config.sumshift - 4);
#if defined(HAVE_SERIAL)
	uart_println("");
#endif	//	#if (defined(DEBUG_TIMERS) || defined(DEBUG_DETECTOR)) && defined(HAVE_SERIAL)

	setleds(0,0,0);

#if defined(HAVE_SERIAL)
	uart_printstr("Sum: ");
	uart_printlong(g_sum);
	uart_println("");
#endif	//	#if (defined(DEBUG_TIMERS) || defined(DEBUG_DETECTOR)) && defined(HAVE_SERIAL)

	while(1)
	{
		while(!g_counter_ready) {
			if( getlinefromserial( g_linebuffer, sizeof( g_linebuffer ), &g_lineidx) ) {
				processinput();
				g_lineidx = 0;
			}
		}

		uint32_t	countercopy = g_timer_counts;
		g_counter_ready = 0;

#if defined(DEBUG_DETECTOR) && defined(HAVE_SERIAL)
		float frq = (countercopy * 1000.0) / g_counting_period;
		unsigned long lf = (unsigned long)frq;
		uart_printstr("Freq: ");
		uart_printlong(lf);
#endif	//	defined(DEBUG_DETECTOR) && defined(HAVE_SERIAL)
#if defined(HAVE_I2C) && defined(USE_I2C)
		int32_t		diff = (int32_t)countercopy - (int32_t)(g_sum >> g_config.sumshift);
		uint16_t	threshold = (g_sum >> g_config.sumshift) / g_config.divider;
		uint8_t		neg = diff < 0, chars;
		if(neg) diff = -diff;
		i2clcd_setcursor(&g_ps,0,1);
		i2clcd_printlong(&g_ps, g_sum >> g_config.sumshift);
		i2clcd_printchar(&g_ps, neg ? '-' : '+');
		i2clcd_printlong(&g_ps, diff);
		i2clcd_printchar(&g_ps, '=');
		i2clcd_printlong(&g_ps, countercopy);
		i2clcd_printchar(&g_ps, ' ');
		chars = diff * 13 / threshold;
		if(chars > 16) chars = 16;
		if(diff && !chars) ++chars;
		i2clcd_setcursor(&g_ps, 0, 0);

		uint8_t	pos = 0;
		if(neg) {
			while(pos<16-chars) {
				i2clcd_printchar(&g_ps, ' ');
				++pos;
			}
			while(chars--) i2clcd_printchar(&g_ps, '<');
		} else {
			while(chars--) {
				i2clcd_printchar(&g_ps, '>');
				++pos;
			}
			while(pos++ < 16) i2clcd_printchar(&g_ps, ' ');
		}
#endif	//	defined(HAVE_I2C) && defined(USE_I2C)

		if((state = detect(countercopy)) >= ACTIVE) {
			setleds(1,1,1);
			PORTB |= _BV(PORT_OUT);
		} else {
			PORTB &= ~_BV(PORT_OUT);
			if(state == BELOW)
				setleds(0,0,1);
			else if(state == BASE)
				setleds(0,0,0);
			else	//ABOVE
				setleds(1,0,0);
		}

#if defined(DEBUG_DETECTOR)
		if(g_badirq && g_badirqcnt == g_badirq) {
			PORTB ^= _BV(PORT_LED2);
			g_badirqcnt = 0;
			++g_badirqcnt;
		}
#endif	//	DEBUG_DETECTOR
	}
}

////////////////////////////////////////////////////////////////////
void setup(uint8_t ms)
{
	init_config();

	g_counting_period = ms;             // how many 1 ms counts to do

	TCCR0A = 0;						//t0: stop
	TCCR0B = 0;
	TCNT0 = 0;						//t0: reset counter

#if defined(__AVR_ATtiny85__)
	TCCR1 = 0;						//t1: stop
	GTCCR |= _BV(PSR1);				//t1: prescaler reset
	TCNT1 = 0;						//t1: reset counter
	OCR1C = 124;					//for counter autoreset
	OCR1A = 124;					//for interrupt

	TIMSK = _BV(TOIE0) | _BV(OCIE1A);	//enable interrupts for t0 & t1

	TCCR1 |= _BV(CS13) | _BV(CTC1);	//t1: prescaler 128, CTC mode (start)
#elif defined(__AVR_ATmega328P__)
	TIMSK0 = _BV(TOIE0);			//t0: overflow interrupt enable

	TCCR2A = 0;						//t2: stop
	TCCR2B = 0;
	GTCCR = _BV(PSRASY);			//t2: prescaler reset
	TCNT2 = 0;						//t2: reset counter

	TCCR2A = _BV(WGM21);			//t2: CTC mode
	OCR2A = 124;					//t2: 62.5*128*125 = 1000000 ns = 1 ms
	TIMSK2 = _BV(OCIE2A);			//t2 enable Timer2 Interrupt

	TCCR2B = _BV(CS20) | _BV(CS22);	//t2: prescaler 128 (start)
#else
#error "Only ATmega 328P and ATtinyX5 are supported."
#endif
	TCCR0B = _BV(CS00) | _BV(CS01) | _BV(CS02);	//t0: ext clk rising edge (start)
#if defined(HAVE_I2C) && defined(USE_I2C)
	i2c_init();
	i2clcd_init(&g_ps, LCD_I2C_ADDRESS);
#endif	//	defined(HAVE_I2C) && defined(USE_I2C)
#if defined(HAVE_SERIAL)
	uart_init(BAUD);
#endif

}  // end of setup

////////////////////////////////////////////////////////////////////
ISR(COUNTERVECT)
{
	++g_counter_overflows;               // count number of Counter1 overflows
}  // end of TIMER1_OVF_vect

////////////////////////////////////////////////////////////////////
//  Timer Interrupt Service is invoked every 1 ms = 1000 Hz
//  16Mhz / 128 / 125 = 1000 Hz
ISR (TIMERVECT)
{
	// grab counter value before it changes any more
	uint8_t	counter_copy = TCNT0;
	uint8_t	tifr_copy = TIFR;

	++g_ms;

	// see if we have reached timing period
	if (++g_timer_overflows < g_counting_period)
		return;  // not yet

	TCNT0 = 0;
	TIFR |= _BV(TOV0);
	g_timer_overflows = 0;

	if(g_counter_ready) {
		g_counter_overflows = 0;
		g_overrun = 1;
		return;
	}

	// if just missed an overflow
#if defined(DEBUG_DETECTOR)
	if(tifr_copy & _BV(TOV0)) {
		g_badirq++;
		g_lastwtf = g_ms;
	}
#endif
	if((tifr_copy & _BV(TOV0)) && counter_copy < 128)
		g_counter_overflows++;


	// end of gate time, measurement ready
	// calculate total count
	g_timer_counts = (g_counter_overflows << 8) + counter_copy; // each overflow is 256 more
	g_counter_ready = 1;              // set global flag for end count period
	g_counter_overflows = 0;
}  // end of TIMER2_COMPA_vect


//////////////////////////////////////////////////////////////////////////////
const char CMD_SHOW[] PROGMEM = "show";
const char CMD_BELOW[] PROGMEM = "below";
const char CMD_BASE[] PROGMEM = "base";
const char CMD_ABOVE[] PROGMEM = "above";
const char CMD_ACTIVE[] PROGMEM = "active";
const char CMD_TIMEDOUT[] PROGMEM = "timedout";
const char CMD_SHIFTSUM[] PROGMEM = "shiftsum";
const char CMD_TIMELIMIT[] PROGMEM = "timelimit";
const char CMD_DIVIDER[] PROGMEM = "divider";
const char CMD_SAVE[] PROGMEM = "save";

const char PARAMERROR[] PROGMEM = "!Parameter error!\r\n";

const char PROGMEM shiftvalues[] = "Diff shift values:\r\n";
const char PROGMEM crlf[] = "\r\n";
const char PROGMEM valsep[] = ": ";
const char*	const shiftnames[] PROGMEM = { CMD_BELOW, CMD_BASE, CMD_ABOVE, CMD_ACTIVE, CMD_TIMEDOUT };

//////////////////////////////////////////////////////////////////////////////
static void printparam(const char *name, long value)
{
	uart_printstr_p(name);
	uart_printstr_p(valsep);
	uart_printlong(value);
	uart_printstr_p(crlf);
}

//////////////////////////////////////////////////////////////////////////////
void processinput()
{
	char	*inptr = (char*) g_linebuffer;

	uart_printchar(CMNT);
	uart_println(inptr);

	if( iscommand(&inptr, CMD_SHOW, 1)) {
		uart_printstr_p(shiftvalues);
		for(uint8_t  c = 0; c < STATECOUNT; ++c ) {
			uart_printchar('\t');
			printparam(pgm_read_ptr(shiftnames + c), g_config.shifts[c]);
		}
		printparam(CMD_SHIFTSUM, g_config.sumshift);
		printparam(CMD_TIMELIMIT, g_config.tlimit);
		printparam(CMD_DIVIDER, g_config.divider);
		uart_printstr_p(crlf);
	}
	else if( iscommand(&inptr, CMD_SHIFTSUM, 1)) {
		long l = getintparam(&inptr, 1, 1, 0);
		if(l > 0 ) g_config.sumshift = l;
		else uart_printstr_p(PARAMERROR);
	}
	else if( iscommand(&inptr, CMD_TIMELIMIT, 1)) {
		long l = getintparam(&inptr, 1, 1, 0);
		if(l > 0 ) g_config.tlimit = l;
		else uart_printstr_p(PARAMERROR);
	}
	else if( iscommand(&inptr, CMD_DIVIDER, 1)) {
		long l = getintparam(&inptr, 1, 1, 0);
		if(l > 0 ) g_config.divider = l;
		else uart_printstr_p(PARAMERROR);
	}
	else if( iscommand(&inptr, CMD_SAVE, 1)) {
		eeprom_update_block((void*)&g_config, (void*)&ee_config, sizeof(g_config));
	}
	else {
		for(uint8_t ds = 0; ds < sizeof(shiftnames)/sizeof(shiftnames[0]); ++ds) {
			if(iscommand(&inptr, pgm_read_ptr(shiftnames + ds), 1)) {
				long l = getintparam(&inptr, 1, 1, 1);
				if(l != LONG_MIN) g_config.shifts[ds] = l;
				else uart_printstr_p(PARAMERROR);
				return;
			}
		}
	}

//	if( iscommand(&inptr,CMD_BASE, 1))
}

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

	uart_printstr(name);
	uart_printstr(": ");
	uart_printstr(buffer);
	uart_printstr(" (0x");
	itoa(value, buffer, 16);
	uart_printstr(buffer);
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

	uart_printstr(name);
	uart_printstr(": ");
	uart_printstr(buffer);
	uart_printstr(" (0x");
	itoa(value, buffer, 16);
	uart_printstr(buffer);
	uart_println(")");
}

////////////////////////////////////////////////////////////////////
void dumpvar32(char *name, unsigned long value)
{
	char buffer[sizeof(value)*8+1];

	uart_printstr(name);
	uart_printstr(": 0x");
	itoa(value, buffer, 16);
	uart_printstr(buffer);
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
	dumpvar32("g_overflowCount", g_counter_overflows);
	dumpvar32("g_timer_counts", g_timerCounts);
	uart_println("");
//	dumpreg("", );
//	dumpreg("", );
//	dumpreg("", );
}
#endif	//	DUMPINFO




