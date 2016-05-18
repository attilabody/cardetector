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

#if defined(__AVR_ATtiny85__)

#define DD_LED 		DDB1
#define DD_OUT		DDB0
#define PORT_LED	PORTB1
#define PORT_OUT	PORTB0
#define TIMERVECT	TIMER1_COMPA_vect
#define COUNTERVECT	TIMER0_OVF_vect

#elif defined(__AVR_ATmega328P__)

#define DD_LED		DDB5
#define DD_OUT		DDB4
#define PORT_LED	PORTB5
#define PORT_OUT	PORTB4
#define TIMERVECT	TIMER2_COMPA_vect
#define COUNTERVECT	TIMER0_OVF_vect
#define	TIFR		TIFR0
#define	HAVE_SERIAL	1

#else
#error "Only ATmega 328P and ATtinyx5 are supported."
#endif

//#define DEBUG_TIMERS
//#define DEBUG_DETECTOR

#endif /* CONFIG_H_ */
