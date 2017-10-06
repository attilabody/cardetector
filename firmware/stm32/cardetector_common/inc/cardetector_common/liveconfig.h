/*
 * liveconfig.h
 *
 *  Created on: Mar 14, 2017
 *      Author: compi
 */

#ifndef CARDETECTOR_COMMON_INC_CARDETECTOR_COMMON_LIVECONFIG_H_
#define CARDETECTOR_COMMON_INC_CARDETECTOR_COMMON_LIVECONFIG_H_

#include <inttypes.h>
#include <cardetector_common/detector.h>

typedef struct
{
	uint8_t		magic;
	int8_t		shifts[STATECOUNT];
	uint8_t		sumshift;
	uint8_t		tlimit;
	uint16_t	thdiv;
	uint8_t		mccount;			//measure cycle count
	uint8_t		debug;
} LIVECONFIG;



#endif /* CARDETECTOR_COMMON_INC_CARDETECTOR_COMMON_LIVECONFIG_H_ */
