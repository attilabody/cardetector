/*
 * config.h
 *
 *  Created on: May 7, 2016
 *      Author: compi
 */

#ifndef CONFIG_H_
#define CONFIG_H_

#define F_CPU 16000000UL
#define BAUD 115200

//#define DEBUG_TIMERS
#define DEBUG_DETECTOR


#if defined(__AVR_ATtiny85__)

#define DD_LED1		DDB1
//#define DD_LED2		DDB4
//#define DD_LED3		DDB3
#define DD_LED2		DDB1
#define DD_LED3		DDB1
#define PORT_LED1	PORTB1
//#define PORT_LED2	PORTB4
//#define PORT_LED3	PORTB3
#define PORT_LED2	PORTB1
#define PORT_LED3	PORTB1
#define DD_OUT		DDB0
#define PORT_OUT	PORTB0
#define PORT_BADIRQ	PORTB4
#define TIMERVECT	TIMER1_COMPA_vect
#define COUNTERVECT	TIMER0_OVF_vect

#elif defined(__AVR_ATmega328P__)

#define DD_LED1		DDB5
#define DD_LED2		DDB4
#define	DD_LED3		DDB3
#define DD_OUT		DDB2
#define PORT_LED1	PORTB5
#define PORT_LED2	PORTB4
#define PORT_LED3	PORTB3
#define PORT_OUT	PORTB2
#define DD_BADIRQ	DDB3
#define PORT_BADIRQ	PORTB3
#define TIMERVECT	TIMER2_COMPA_vect
#define COUNTERVECT	TIMER0_OVF_vect
#define	TIFR		TIFR0
#define	HAVE_SERIAL	1

#else
#error "Only ATmega 328P and ATtinyx5 are supported."
#endif

#define SHIFT_BELOW		5
#define SHIFT_BASE		0	//not used
#define SHIFT_ABOVE		3
#define SHIFT_ACTIVE	0
#define SHIFT_TIMEOUT	8

#define SHIFT_SUM		8
#define	DIVIDER			300
#define	SHIFT_TOLERANCE	3

#define TIMELIMIT		60
#endif /* CONFIG_H_ */
