/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"
#include "i2c.h"
#include "iwdg.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include <config.h>
#include <globals.h>
#include <cardetector_common/detector.h>
#include <cardetector_common/display.h>
#include <stm32plus/usart.h>
#include <stm32plus/strutil.h>

#include <cardetector_common/liveconfig.h>
#include <cardetector_common/userinput.h>

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

const char g_stateSyms[] = "-|+#*";

LIVECONFIG 		g_config =
{
	  0xA5
	, {SHIFT_BELOW, SHIFT_BASE, SHIFT_ABOVE, SHIFT_ACTIVE, SHIFT_TIMEOUT}	//below, above, active, timeout
	, SHIFT_SUM		//shift
	, TIMELIMIT		//tlimit
	, DIVIDER		//divider
	, MCCOUNT		//mccntexp
};

DETECTORSTATUS g_statuses[2] = {
		{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, BASE, ACTIVE1_GPIO_Port, ACTIVE1_Pin},
		{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, BASE, ACTIVE2_GPIO_Port, ACTIVE2_Pin}
};

// globals.h start /////////////////////////////////////////////////////////////
I2cMaster_State		*g_i2c;
I2cEEPROM_State		g_eeprom;

uint8_t				g_lineBuffer[64];
volatile uint8_t	g_lineReceived = 0;

// globals.h end ///////////////////////////////////////////////////////////////

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void DisplayResults(uint8_t line);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
#ifdef GENERATE_SWEEP
#define PWM_DELAY	2
#define PWM_STEP	1

//////////////////////////////////////////////////////////////////////////////
void HAL_SYSTICK_Callback(void)
{
	static uint16_t	counter = PWM_DELAY;
	static uint8_t	up = 1;
	static uint16_t	pwm = 0;

	//return;

	if(! --counter)
	{
		counter = PWM_DELAY;

		if(up) {
			if(pwm < htim14.Init.Period) {
				pwm += PWM_STEP;
			} else {
				pwm = htim14.Init.Period;
				up = 0;
			}
		} else {
			if(pwm > PWM_STEP) {
				pwm -= PWM_STEP;
			} else {
				pwm = 0;
				up = 1;
			}
		}
		__HAL_TIM_SetCompare(&htim14, TIM_CHANNEL_1, pwm);
	}
}
#endif	//	GENERATE_SWEEP

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
#ifndef USE_SERIAL
#undef DEBUG_SERIAL
#define MX_USART1_UART_Init()
#endif

#ifndef USE_I2C
#undef USE_LCD
#undef USE_LEDBAR
#define MX_I2C1_Init()
#endif


  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM16_Init();
  MX_TIM17_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
#ifdef USE_I2C
  g_i2c = I2cMaster_Init(&hi2c1);

  InitializeDisplay(g_i2c);

  MX_IWDG_Init();
  HAL_IWDG_Refresh(&hiwdg);

  I2cEEPROM_Init(&g_eeprom, g_i2c, EEPROMADDR, 2, 32);
  {
	LIVECONFIG	config;

	if(I2cEEPROM_Read(&g_eeprom, EESTART, &config, sizeof(config)) == HAL_OK) {
		I2cMaster_WaitCallback(g_i2c);
		if(config.magic == 0xA5)
			g_config = config;
	}
  }
#endif

#ifdef USE_SERIAL
  UsartInit(&huart1);
#endif	//	USE_SERIAL

#ifdef GENERATE_SWEEP
  HAL_TIM_Base_Start_IT(&htim14);
  HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1);
  __HAL_TIM_SetCompare(&htim14, TIM_CHANNEL_1, 256);
#endif	//	GENERATE_SWEEP

  HAL_TIM_IC_Start_IT(&htim16, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim17, TIM_CHANNEL_1);

#ifdef USE_SERIAL
  HAL_UART_Receive_IT(&huart1, g_lineBuffer, sizeof(g_lineBuffer));
#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
#ifdef USE_SERIAL
	if(g_lineReceived)
	{
	  UsartSendStr((char*)g_lineBuffer, 1);
	  UsartSendStr("\r\n", 1);

	  ProcessInput(&g_config, (char*)g_lineBuffer);
	  //DisplayInput(&i2clcd);
	  g_lineReceived = 0;
	  HAL_UART_Receive_IT(&huart1, g_lineBuffer, sizeof(g_lineBuffer));
	}
#endif
	  if(g_statuses[0].trigger) {
		  DisplayResults(0);
		  g_statuses[0].trigger = 0;
	  }
	  if(g_statuses[1].trigger) {
		  DisplayResults(1);
		  g_statuses[1].trigger = 0;
	  }

	  HAL_IWDG_Refresh(&hiwdg);
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL3;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
////////////////////////////////////////////////////////////////////
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart1)
	{
		g_lineBuffer[sizeof(g_lineBuffer) - 1 - huart->RxXferCount] = 0;
		g_lineReceived = 1;
	}
}

////////////////////////////////////////////////////////////////////
void DisplayResults(uint8_t line)
{
	uint8_t			irqEnabled = __get_PRIMASK() == 0;
	__disable_irq();
	DETECTORSTATUS	st = g_statuses[line];
	if(irqEnabled) __enable_irq();

#ifdef DEBUG_SERIAL
	if(g_config.debug & (line+1))
	{
		// index state sum avg lastMeasured diff tolerance threshold correction
		UsartPrintUint(line, 0, 1);
		UsartSend(": ", 2, 1);
		UsartSend(&g_stateSyms[st.state], 1, 1);
		UsartSend(" ", 1, 1);
		UsartPrintUint(st.sum, -1, 1);
		UsartSend(", ", 2, 1);
		UsartPrintUint(st.avg, -1, 1);
		UsartSend(", ", 2, 1);
		UsartPrintUint(st.lastMeasured, -1, 1);
		UsartSend(", ", 2, 1);
		UsartPrintInt(st.diff, 0, 1);
		UsartSend(", ", 2, 1);
		UsartPrintUint(st.tolerance, 0, 1);
		UsartSend(", ", 2, 1);
		UsartPrintUint(st.threshold, 0, 1);
		UsartSend(", ", 2, 1);
		UsartPrintInt(st.correction, 0, 1);
		UsartSend("\r\n", 2, 1);
	}
#endif	//	DEBUG_SERIAL

#if defined(USE_LCD) && defined(DEBUG_LCD)
	I2cLcd_SetCursor(&g_lcd, 0, line);
	I2cLcd_PrintChar(&g_lcd, g_stateSyms[st.state]);
	I2cLcd_PrintChar(&g_lcd, ' ');
	I2cLcd_PrintInt(&g_lcd, st.diff, 0);
	I2cLcd_PrintStr(&g_lcd, "   ");
#endif	//	USE_LCD

#if defined(USE_LCD) || defined(USE_LEDBAR)
	UpdateBar(line, CalcBar(&st));
#endif
}

//TODO: REMOVE DEBUG CODE
//volatile uint32_t g_basz = 0;

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

//TODO: REMOVE DEBUG CODE
//			if(lastDelta > 0xffff)
//				++ g_basz;

			//HAL_GPIO_TogglePin(ACTIVE2_GPIO_Port, ACTIVE2_Pin);

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

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
