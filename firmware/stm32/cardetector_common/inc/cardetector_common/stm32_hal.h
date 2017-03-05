#pragma once
#ifndef _CDC_STM32HAL_H_INCLUDED
#define _CDC_STM32HAL_H_INCLUDED
#if defined(STM32F030x6)
#	include "stm32f0xx_hal.h"
#	define STM32F0XX
#elif defined(STM32F103xB)
#	include "stm32f1xx_hal.h"
#	define STM32F1XX
#endif

#endif //	_CDC_STM32HAL_H_INCLUDED
