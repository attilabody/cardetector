/*
 * detector.h
 *
 *  Created on: Mar 15, 2017
 *      Author: compi
 */

#pragma once
#include <inttypes.h>
#include <stm32plus/stm32_hal.h>

enum INTERNALSTATES
{
	BELOW = 0,
	BASE,
	ABOVE,
	ACTIVE,
	TIMEOUT,
	STATECOUNT
};

typedef struct
{
	volatile uint8_t	trigger;
	uint32_t			sum;
	uint32_t			accumulator;
	uint32_t			lastMeasured;
	uint32_t			activeStart;
	int32_t				correction;
	int16_t				diff;
	uint16_t			tolerance;
	uint16_t			threshold;
	uint16_t			avg;
	uint16_t			prevCapture;
	uint8_t				initialized;
	uint8_t				counter;
	enum INTERNALSTATES	state;
	const GPIO_TypeDef*	port;
	const uint16_t		pin;
	//uint32_t	debug[MCCOUNT];
} DETECTORSTATUS;

