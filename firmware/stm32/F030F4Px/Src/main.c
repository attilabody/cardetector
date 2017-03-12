/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include <string.h>
#include "config.h"
#include <stm32plus/usart.h>
#include <stm32plus/i2clcd.h>
#include <stm32plus/i2ceeprom.h>
#include <stm32plus/strutil.h>

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define LCDADDR (0x27 << 1)

enum STATES
{
	BELOW = 0,
	BASE,
	ABOVE,
	ACTIVE,
	TIMEOUT,
	STATECOUNT
};

const char g_stateSyms[] = "-|+#*";
typedef struct
{
	uint8_t		magic;
	int8_t		shifts[STATECOUNT];
	uint8_t		sumshift, tlimit;
	uint16_t	divider;
	uint8_t		mccount;			//measure cycle count
} CONFIG;

CONFIG 		g_config =
{
	  0xA5
	, {SHIFT_BELOW, SHIFT_BASE, SHIFT_ABOVE, SHIFT_ACTIVE, SHIFT_TIMEOUT}	//below, above, active, timeout
	, SHIFT_SUM		//shift
	, TIMELIMIT		//tlimit
	, DIVIDER		//divider
	, MCCOUNT		//mccntexp
};

typedef struct
{
	uint32_t	sum;
	uint32_t	accumulator;
	uint32_t	lastMeasured;
	uint32_t	activeStart;
	int32_t		correction;
	int16_t		diff;
	uint16_t	tolerance;
	uint16_t	threshold;
	uint16_t	avg;
	uint16_t	prevCapture;
	uint8_t		initialized;
	uint8_t		counter;
	enum STATES	state;
	//uint32_t	debug[MCCOUNT];
} CHANNELSTATUS;

CHANNELSTATUS g_statuses[2] = {
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, BELOW},
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, BELOW}
};

static const char	g_wrbuf[] = "0123456789ABCDEF";
uint8_t				g_rdbuf[16];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void DisplayResults(I2cLcd_State *i2clcd, uint8_t line);

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
  I2cMaster_State		*i2c;
#ifdef USE_LCD
  I2cLcd_State			i2clcd;
#endif //	USE_LCD
  I2cEEPROM_State		i2ceeprom;
  volatile uint32_t		i2cErr = HAL_I2C_ERROR_NONE;

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM16_Init();
  MX_TIM17_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
  i2c = I2cMaster_Init(&hi2c1);
#ifdef USE_LCD
  I2cLcd_Init(&i2clcd, i2c, LCDADDR );
  I2cLcd_InitDisplay(&i2clcd);
#endif	//	USE_LCD

#define EESTART	0

  memset(g_rdbuf, 0, sizeof(g_rdbuf));
  I2cEEPROM_Init(&i2ceeprom, i2c, EEPROMADDR, 2, 32);
  i2cErr = I2cEEPROM_Read(&i2ceeprom, EESTART, g_rdbuf, sizeof(g_rdbuf));
  i2cErr = I2cEEPROM_Write(&i2ceeprom, EESTART, g_wrbuf, sizeof(g_wrbuf)-1);
  memset(g_rdbuf, 0, sizeof(g_rdbuf));
  i2cErr = I2cEEPROM_Read(&i2ceeprom, EESTART, g_rdbuf, sizeof(g_rdbuf));
  i2cErr = I2cEEPROM_Read(&i2ceeprom, EESTART+2, g_rdbuf, sizeof(g_rdbuf));
  I2cMaster_WaitCallback(i2c);

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
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	  HAL_Delay(100);
#ifdef USE_LCD
	  DisplayResults(&i2clcd, 0);
#else
	  DisplayResults(NULL, 0);
#endif
	  //DisplayResults(&i2clcd, 1);

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
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
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
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
void DisplayResults(I2cLcd_State *i2clcd, uint8_t line)
{
	uint8_t			irqEnabled = __get_PRIMASK() == 0;
	__disable_irq();
	CHANNELSTATUS	st = g_statuses[line];
	if(irqEnabled) __enable_irq();

#ifdef USE_SERIAL
	UsartSend(&g_stateSyms[st.state], 1, 1);
	UsartSend(" ", 1, 1);
	UsartSendUint(st.sum, 1, 1);
	UsartSend(", ", 2, 1);
	UsartSendUint(st.avg, 1, 1);
	UsartSend(", ", 2, 1);
	UsartSendUint(st.lastMeasured, 1, 1);
	UsartSend(", ", 2, 1);
	UsartSendInt(st.diff, 0, 1);
	UsartSend(", ", 2, 1);
	UsartSendUint(st.tolerance, 0, 1);
	UsartSend(", ", 2, 1);
	UsartSendUint(st.threshold, 0, 1);
	UsartSend(", ", 2, 1);
	UsartSendInt(st.correction, 0, 1);
	UsartSend("\r\n", 2, 1);
#endif	//	USE_SERIAL

#ifdef USE_LCD
	I2cLcd_SetCursor(i2clcd, 0, 0);
	I2cLcd_PrintChar(i2clcd, g_stateSyms[st.state]);
	I2cLcd_PrintChar(i2clcd, ' ');
	I2cLcd_PrintInt(i2clcd, st.diff, 0);
	I2cLcd_PrintStr(i2clcd, "   ");
#endif	//	USE_LCD
}

volatile uint32_t g_basz = 0;
//////////////////////////////////////////////////////////////////////////////
void detect(CHANNELSTATUS *st, TIM_HandleTypeDef *htim)
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
			if(lastDelta > 0xffff)
				++ g_basz;
			if(!st->sum)
				st->sum = ((uint32_t)lastDelta * g_config.mccount) << g_config.sumshift;
			else
			{
				//HAL_GPIO_TogglePin(ACTIVE2_GPIO_Port, ACTIVE2_Pin);

				st->accumulator += lastDelta;
				//st->debug[st->counter] = lastDelta;
				if(++st->counter == g_config.mccount)
				{
					//HAL_GPIO_TogglePin(ACTIVE1_GPIO_Port, ACTIVE1_Pin);

					st->lastMeasured = st->accumulator;
					st->accumulator = 0;
					st->counter = 0;

					st->avg = st->sum >> g_config.sumshift;
					st->threshold = st->avg / g_config.divider;
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
					} else if(HAL_GetTick() - st->activeStart > TIMELIMIT * 1000)
						st->state = TIMEOUT;

					shift = g_config.shifts[st->state];
					if(shift != SCHAR_MIN) {
						st->correction = (shift >= 0) ? (((int32_t)st->diff) << shift) : (((int32_t)st->diff) >> -shift);
						st->sum += st->correction;
					} else
						st->correction = 0;
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
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
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
