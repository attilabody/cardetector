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
#include <avr/wdt.h>

#include <stdlib.h>
#include <limits.h>

#include "serial.h"
#include "i2c.h"
#include "pcf8574.h"
#include "i2c_lcd.h"
#include "utils.h"
#include "commsyms.h"
#include "setup.h"

#if defined(DEBUG_DETECTOR) && defined(HAVE_SERIAL)
volatile uint16_t	g_badirq = 0;
uint16_t			g_badirqcnt = 0;
#endif

volatile uint16_t	g_timer_counts;
volatile uint8_t	g_counter_ready = 0;
volatile uint8_t	g_overrun = 0;
volatile uint32_t	g_ms = 0;
#if defined(DEBUG_DETECTOR) && defined(HAVE_SERIAL)
volatile uint32_t	g_lastwtf = 0;
#endif

volatile uint8_t	g_counter_overflows = 0;
volatile uint16_t	g_timer_overflows = 0;
uint16_t 			g_counting_period = 250;
uint16_t			g_active_time = 0;

#if defined(HAVE_SERIAL)
unsigned char 	g_linebuffer[64];
unsigned char	g_lineidx;
uint8_t			g_debug;
#endif

uint16_t		g_threshold = 0;
int16_t			g_diff = 0;
uint16_t		g_avg;
int16_t			g_tolerance;

#if defined(HAVE_I2C) && defined(USE_I2C)
PCF8574_STATUS	g_ps;
#endif	//	defined(HAVE_I2C) && defined(USE_I2C)

#if defined(HAVE_SERIAL)
void processinput();
#endif

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
	uint8_t		magic;
	int8_t		shifts[5];
	uint8_t		sumshift, tlimit;
	uint16_t	divider;
} CONFIG;

CONFIG EEMEM ee_config =
{
	  0xA5
	, {SHIFT_BELOW, SHIFT_BASE, SHIFT_ABOVE, SHIFT_ACTIVE, SHIFT_TIMEOUT}	//below, above, active, timeout
	, SHIFT_SUM		//shift
	, TIMELIMIT		//tlimit
	, DIVIDER		//divider
};

CONFIG 		g_config =
{
	  0xA5
	, {SHIFT_BELOW, SHIFT_BASE, SHIFT_ABOVE, SHIFT_ACTIVE, SHIFT_TIMEOUT}	//below, above, active, timeout
	, SHIFT_SUM		//shift
	, TIMELIMIT		//tlimit
	, DIVIDER		//divider
};

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
void init_config()
{
	if(eeprom_read_byte((uint8_t*)&ee_config) == 0xA5)
		eeprom_read_block((void*)&g_config, &ee_config, sizeof(g_config));
}

////////////////////////////////////////////////////////////////////
enum STATES detect(uint16_t counter)
{
#if defined(DEBUG_DETECTOR) && defined(HAVE_SERIAL)
	uint16_t		debugavg = g_sum >> g_config.sumshift;
	int32_t			modifier;
#endif
	int8_t			shift;

	enum STATES		state;

	if (g_diff < 0)
		state = g_tolerance + g_diff < 0 ? BELOW : BASE;
	else if( g_diff < g_tolerance)
		state = BASE;
	else if(g_diff < g_threshold)
		state = ABOVE;
	else if (g_active_time < ((uint16_t)g_config.tlimit) << 2)
		state = ACTIVE;
	else
		state = TOUT;

	shift = g_config.shifts[state];
	if(shift != SCHAR_MIN)
		g_sum += (shift >= 0) ? (((int32_t)g_diff) << shift) : (((int32_t)g_diff) >> -shift);

#if defined(DEBUG_DETECTOR) && defined(HAVE_SERIAL)
	if(g_debug)
	{
		modifier = (shift >= 0) ? (g_diff << shift) : (g_diff >> -shift);
		uart_printstr(" Sum: ");
		uart_printlong(g_sum);
		uart_printstr(" Raw: ");
		uart_printlong(g_timer_counts);
		uart_printstr(" Avg: ");
		uart_printlong(debugavg);
		uart_printstr(" Diff: ");
		uart_printlong(g_diff);
		uart_printstr(" Sta: ");
		uart_printlong(state);
		uart_printstr(" << : ");
		uart_printlong(g_config.shifts[state]);
		uart_printstr(" Mod: ");
		uart_printlong(modifier);
		uart_printstr(" Th: ");
		uart_printlong(g_threshold);
		uart_printstr(" To: ");
		uart_printlong(g_tolerance);
		uart_printstr(" BAD: ");
		uart_printlong(g_badirq);

		uart_println("");
	}
#endif	//	DEBUG_DETECTOR

	if(state < ACTIVE) g_active_time = 0;
	else ++g_active_time;

	return state;
}

////////////////////////////////////////////////////////////////////
#ifdef __AVR_ATmega328P__
PORTBIT g_leds[] = {
	  { PORT_LED1, _BV(BIT_LED1) }
	, { PORT_LED2, _BV(BIT_LED2) }
	, { PORT_LED3, _BV(BIT_LED3) }
	, { PORT_LED4, _BV(BIT_LED4) }
	, { PORT_LED5, _BV(BIT_LED5) }
	, { PORT_LED6, _BV(BIT_LED6) }
	, { PORT_LED7, _BV(BIT_LED7) }
	, { PORT_LED8, _BV(BIT_LED8) }
};
#endif

////////////////////////////////////////////////////////////////////
void updateledbar(int8_t value)	//-2 ... 6
{
#if defined(__AVR_ATmega328P__)
	int8_t	led;
	for(led = -2; led < 0; ++led) {
		if(value <= led) _SFR_IO8(g_leds[led+2].port) |= g_leds[led+2].bitmask;
		else _SFR_IO8(g_leds[led+2].port) &= ~g_leds[led+2].bitmask;
	}

	for(led = 0; led < 6; ++led) {
		if(led < value) _SFR_IO8(g_leds[led+2].port) |= g_leds[led+2].bitmask;
		else _SFR_IO8(g_leds[led+2].port) &= ~g_leds[led+2].bitmask;
	}
#elif defined(__AVR_ATtiny85__)
	if(value) PORT_LED1 |= _BV(BIT_LED1);
	else PORT_LED1 &= ~_BV(BIT_LED1);
#endif
}

////////////////////////////////////////////////////////////////////
int8_t calcbar(const uint8_t top, uint8_t limit)
{
	if(!g_diff) return 0;

	uint8_t	neg = g_diff < 0;
	int16_t	absdiff = !neg ? g_diff : -g_diff;
	int16_t	ret = 0;

	if(absdiff < g_tolerance) return 0;

	absdiff -= g_tolerance;
	int16_t	threshold = g_threshold - g_tolerance;

	ret = ((int32_t)absdiff * (top - 1))/threshold + 1;
	if(ret > limit) ret = limit;
	return neg ? -ret : ret;
}

////////////////////////////////////////////////////////////////////
inline void __attribute__((always_inline)) calculatemetrics(uint16_t counter)
{
	g_threshold = (g_sum >> g_config.sumshift) / g_config.divider;
	g_avg = g_sum >> g_config.sumshift;
	g_diff = (int32_t)counter - g_avg;
	g_tolerance = g_threshold >> SHIFT_TOLERANCE;
}

////////////////////////////////////////////////////////////////////
inline void __attribute__((always_inline)) setoutput(uint8_t value)
{
	if(value) PORT_OUT |= _BV(BIT_OUT);
	else PORT_OUT &= ~_BV(BIT_OUT);
}


////////////////////////////////////////////////////////////////////
#if defined(HAVE_I2C) && defined(USE_I2C)
void updatelcdbar(int8_t chars)
{
	i2clcd_setcursor(&g_ps, 0, 0);
	int8_t pos;
	if (chars < 0) {
		pos = -16;
		while (pos < chars) {
			i2clcd_printchar(&g_ps, ' ');
			++pos;
		}
		while (chars++ < 0)
			i2clcd_printchar(&g_ps, '<');
	} else {
		pos = 0;
		while (chars--) {
			i2clcd_printchar(&g_ps, '>');
			++pos;
		}
		while (pos++ < 16)
			i2clcd_printchar(&g_ps, ' ');
	}
}
#else
static inline void updatelcdbar(int8_t chars) {}
#endif	//	defined(HAVE_I2C) && defined(USE_I2C)

////////////////////////////////////////////////////////////////////
void updatedisplays(uint16_t counter, enum STATES state )
{
#if defined(HAVE_I2C) && defined(USE_I2C)
	uint8_t	neg = g_diff < 0;
	int16_t absdiff = neg ? -g_diff : g_diff;
	int8_t	chars = calcbar(13,16);

	i2clcd_setcursor(&g_ps,0,1);
	i2clcd_printlong(&g_ps, g_sum >> g_config.sumshift);
	i2clcd_printchar(&g_ps, neg ? '-' : '+');
	i2clcd_printlong(&g_ps, absdiff);
	i2clcd_printchar(&g_ps, '=');
	i2clcd_printlong(&g_ps, counter);
	i2clcd_printchar(&g_ps, ' ');

	updatelcdbar(chars);
#endif	//	defined(HAVE_I2C) && defined(USE_I2C)
	updateledbar(calcbar(5,6));
}

////////////////////////////////////////////////////////////////////
void calibrate()
{
	uart_printstr("Calibrating: ");
	for (uint8_t count = 0; count < 2; ++count)
	{
		wdt_reset();
		while (!g_counter_ready)
			;
		g_counter_ready = 0;
		updatelcdbar((count&1) ? -16 : 16 );
		updateledbar((count&1) ? 0 : 6 );

		uart_printchar('x');
	}
	for (uint8_t count = 0; count < (1 <<CALIBRATE_LOOP_POW); ++count)
	{
		wdt_reset();
		while (!g_counter_ready)
			;
		g_sum += g_timer_counts;
		g_counter_ready = 0;
		updatelcdbar((count&1) ? -16 : 16 );
		updateledbar((count&1) ? -0 : 6 );

		uart_printchar('.');
	}
	g_sum <<= (g_config.sumshift - 2);
	uart_printstr("\r\nSum: ");
	uart_printlong(g_sum);
	uart_println("");
}

////////////////////////////////////////////////////////////////////
int main(void)
{
	enum STATES		state;

	init_config();
	setup();
#if defined(HAVE_I2C) && defined(USE_I2C)
	i2c_init();
	i2clcd_init(&g_ps, LCD_I2C_ADDRESS);
#endif	//	defined(HAVE_I2C) && defined(USE_I2C)
#if defined(HAVE_SERIAL)
	uart_init(BAUD);
#endif

	sei();
	calibrate();

	wdt_enable(WDTO_500MS);
	while(1)
	{
		wdt_reset();
		while(!g_counter_ready) {
#if defined(HAVE_SERIAL)
			if( getlinefromserial( g_linebuffer, sizeof( g_linebuffer ), &g_lineidx) ) {
				processinput();
				g_lineidx = 0;
			}
#endif
		}

		uint16_t	countercopy = g_timer_counts;
		g_counter_ready = 0;

#if defined(DEBUG_DETECTOR) && defined(HAVE_SERIAL)
		if(g_debug)
		{
			float frq = ((uint32_t)countercopy * 1000.0) / g_counting_period;
			uart_printstr("Freq: ");
			uart_printlong((unsigned long)frq);
		}
#endif	//	defined(DEBUG_DETECTOR) && defined(HAVE_SERIAL)
		calculatemetrics(countercopy);
		state = detect(countercopy);
		setoutput(state == ACTIVE);
		updatedisplays(countercopy, state);

#if defined(DEBUG_DETECTOR) && defined(HAVE_SERIAL)
		if(g_badirq && g_badirqcnt == g_badirq) {
			PORTB ^= _BV(PORT_LED2);
			g_badirqcnt = 0;
			++g_badirqcnt;
		}
#endif	//	DEBUG_DETECTOR
	}
}

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
#if defined(DEBUG_DETECTOR) && defined(HAVE_SERIAL)
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
const char CMD_DEBUG[] PROGMEM = "debug";
const char CMD_INFINITELOOP[] PROGMEM = "il";

const char PARAMERROR[] PROGMEM = "!Parameter error!\r\n";

const char PROGMEM shiftvalues[] = "Diff shift values:\r\n";
const char PROGMEM crlf[] = "\r\n";
const char PROGMEM valsep[] = ": ";
const char*	const shiftnames[] PROGMEM = { CMD_BELOW, CMD_BASE, CMD_ABOVE, CMD_ACTIVE, CMD_TIMEDOUT };

//////////////////////////////////////////////////////////////////////////////
#if defined(HAVE_SERIAL)
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
	else if( iscommand(&inptr, CMD_DEBUG, 1)) {
		long l = getintparam(&inptr, 1, 1, 0);
		g_debug = l != 0;
	}
	else if( iscommand(&inptr, CMD_INFINITELOOP, 1)) {
		while(1) {
			uart_printchar('.');
			_delay_ms(250);
		}
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
		uart_println("Unknown command.");
	}
}
#endif

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




