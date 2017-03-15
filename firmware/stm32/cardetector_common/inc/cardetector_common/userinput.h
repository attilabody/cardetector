/*
 * userinput.h
 *
 *  Created on: Mar 14, 2017
 *      Author: compi
 */

#pragma once
#ifndef APPLICATION_USER_USERINPUT_H_
#define APPLICATION_USER_USERINPUT_H_

#include <stm32plus/stm32_hal.h>
#include <cardetector_common/liveconfig.h>
#include <globals.h>

void ProcessInput(LIVECONFIG *config, char const *buffer);

#endif /* APPLICATION_USER_USERINPUT_H_ */
