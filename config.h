/*
 * config.h
 *
 *  Created on: May 7, 2016
 *      Author: compi
 */

#ifndef CONFIG_H_
#define CONFIG_H_

#include <limits.h>

#define F_CPU 16000000UL
#define BAUD 115200

#define USE_I2C

//#define DEBUG_TIMERS
//#define DEBUG_DETECTOR
//#define DEBUG_SERIALIN


#if defined(__AVR_ATtiny85__)
#define DD_LED1		DDB1
#define PORT_LED1	PORTB
#define BIT_LED1	PORTB1

#define DDB_OUT		DDB0
#define PORT_OUT	PORTB
#define BIT_OUT		PORTB0

#define PORT_BADIRQ	PORTB4
#define TIMERVECT	TIMER1_COMPA_vect
#define COUNTERVECT	TIMER0_OVF_vect

#elif defined(__AVR_ATmega328P__)
typedef struct {
	unsigned char	port;
	unsigned char	bitmask;
} PORTBIT;

#define DDB_LED1	DDB2
#define DDB_LED2	DDB1
#define	DDB_LED3	DDD7
#define	DDB_LED4	DDD6
#define	DDB_LED5	DDC3
#define	DDB_LED6	DDC2
#define	DDB_LED7	DDC1
#define	DDB_LED8	DDC0

#define DDR_LED1	DDRB
#define DDR_LED2	DDRB
#define	DDR_LED3	DDRD
#define	DDR_LED4	DDRD
#define	DDR_LED5	DDRC
#define	DDR_LED6	DDRC
#define	DDR_LED7	DDRC
#define	DDR_LED8	DDRC

#define BIT_LED1	PORTB2
#define BIT_LED2	PORTB1
#define BIT_LED3	PORTD7
#define BIT_LED4	PORTD6
#define BIT_LED5	PORTC3
#define BIT_LED6	PORTC2
#define BIT_LED7	PORTC1
#define BIT_LED8	PORTC0

#define PORT_LED1	0x05	//PORTB
#define PORT_LED2	0x05
#define PORT_LED3	0x0B	//PORTD
#define PORT_LED4	0x0B
#define PORT_LED5	0x08	//PORTC
#define PORT_LED6	0x08
#define PORT_LED7	0x08
#define PORT_LED8	0x08

#define DDB_OUT		DDB0
#define PORT_OUT	PORTB
#define BIT_OUT		PORTB0

#define TIMERVECT	TIMER2_COMPA_vect
#define COUNTERVECT	TIMER0_OVF_vect
#define	TIFR		TIFR0
#define	HAVE_SERIAL	1
#define HAVE_I2C	1

#else
#error "Only ATmega 328P and ATtinyx5 are supported."
#endif

#define SHIFT_BELOW		5
#define SHIFT_BASE		0	//not used
#define SHIFT_ABOVE		3
#define SHIFT_ACTIVE	SCHAR_MIN
#define SHIFT_TIMEOUT	8

#define SHIFT_SUM		8
#define	DIVIDER			800
#define	SHIFT_TOLERANCE	3

#define TIMELIMIT		180

#define LCD_I2C_ADDRESS	0x27

#endif /* CONFIG_H_ */
