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

#define DD_LED 		DDB1
#define DD_OUT		DDB0
#define PORT_LED	PORTB1
#define PORT_OUT	PORTB0

#define TIMERVECT	TIMER1_COMPA_vect
#define COUNTERVECT	TIMER0_OVF_vect

//#define DEBUG_TIMERS
//#define DEBUG_DETECTOR

#endif /* CONFIG_H_ */
