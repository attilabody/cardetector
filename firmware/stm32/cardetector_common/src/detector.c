#include "stm32f0xx_hal.h"
#include "tim.h"
#include "gpio.h"

#include <limits.h>
#include <config.h>
#include <globals.h>
#include <cardetector_common/detector.h>
#include <cardetector_common/display.h>
#include <stm32plus/usart.h>
#include <stm32plus/strutil.h>

#include <cardetector_common/liveconfig.h>
#include <cardetector_common/userinput.h>

//////////////////////////////////////////////////////////////////////////////
DETECTORSTATUS g_statuses[2] = {
		{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, BASE, ACTIVE1_GPIO_Port, ACTIVE1_Pin},
		{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, BASE, ACTIVE2_GPIO_Port, ACTIVE2_Pin}
};


//////////////////////////////////////////////////////////////////////////////
void detect(DETECTORSTATUS *st, TIM_HandleTypeDef *htim)
{
	int8_t		shift;
	int16_t		diff;
	uint16_t	lastDelta;
	uint16_t	capturedValue;

	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
	{
		capturedValue = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
		if(st->initialized)
		{
			lastDelta = capturedValue - st->prevCapture;

			st->accumulator += lastDelta;
			//st->debug[st->counter] = lastDelta;
			if(++st->counter == g_config.mccount)
			{
				//HAL_GPIO_TogglePin(ACTIVE1_GPIO_Port, ACTIVE1_Pin);

				st->lastMeasured = st->accumulator;
				st->accumulator = 0;
				st->counter = 0;

				if(!st->sum)
					st->sum = (uint32_t)st->lastMeasured << g_config.sumshift;
				else
				{
					st->avg = st->sum >> g_config.sumshift;
					st->threshold = st->avg / g_config.thdiv;
					st->diff = (int32_t)st->lastMeasured - st->avg;
					st->tolerance = st->threshold >> SHIFT_TOLERANCE;

					diff = -st->diff;

					if (diff < 0)
						st->state = st->tolerance + diff < 0 ? BELOW:BASE;
					else if(diff < st->tolerance)
						st->state = BASE;
					else if(diff < st->threshold)
						st->state = ABOVE;
					else if(st->state < ACTIVE) {	//!ACTIVE && !TIMEOUT
						st->activeStart = HAL_GetTick();
						st->state = ACTIVE;
					} else if(HAL_GetTick() - st->activeStart > g_config.tlimit * 1000)
						st->state = TIMEOUT;

					shift = g_config.shifts[st->state];
					if(shift != SCHAR_MIN) {
						st->correction = (shift >= 0) ? (((int32_t)st->diff) << shift) : (((int32_t)st->diff) >> -shift);
						st->sum += st->correction;
					} else
						st->correction = 0;

					HAL_GPIO_WritePin((GPIO_TypeDef*)st->port, st->pin, st->state < ACTIVE ? GPIO_PIN_RESET : GPIO_PIN_SET);
					st->trigger = 1;
				}
			}
		} else
			st->initialized = 1;

		st->prevCapture = capturedValue;
	}

}

//////////////////////////////////////////////////////////////////////////////
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if( htim->Instance == TIM16)
		detect(&g_statuses[0], htim);
	else if( htim->Instance == TIM17)
		detect(&g_statuses[1], htim);
}

//////////////////////////////////////////////////////////////////////////////
