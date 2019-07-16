/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "iwdg.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
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

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

const char g_stateSyms[] = "-|+#*";

const LIVECONFIG g_config_default =
{
	  0xA5
	, {SHIFT_BELOW, SHIFT_BASE, SHIFT_ABOVE, SHIFT_ACTIVE, SHIFT_TIMEOUT}	//below, above, active, timeout
	, SHIFT_SUM		//shift
	, TIMELIMIT		//tlimit
	, DIVIDER		//divider
	, MCCOUNT		//mccntexp
	, 0
#if defined(USE_LEDBAR)
	, {
		{ 1, 2, 6, 0xe, 0x1e, 0x3e, 0x7e, 0xfe },
		{ 1, 2, 6, 0xe, 0x1e, 0x3e, 0x7e, 0xfe }
	  }
#endif
};

LIVECONFIG 	g_config;

// globals.h start /////////////////////////////////////////////////////////////
#ifdef USE_I2C
I2cMaster_State		*g_i2c;
#endif

#ifdef USE_EEPROM
I2cEEPROM_State		g_eeprom;
#endif

#ifdef USE_SERIAL
uint8_t				g_lineBuffer[64];
volatile uint8_t	g_lineReceived = 0;
#endif

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void DisplayResults(uint8_t line);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
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

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

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
  g_config = g_config_default;

#ifdef USE_I2C
  g_i2c = I2cMaster_Init(&hi2c1);
  InitializeDisplay(g_i2c);
#endif

  MX_IWDG_Init();
  HAL_IWDG_Refresh(&hiwdg);

#ifdef USE_EEPROM
  I2cEEPROM_Init(&g_eeprom, g_i2c, EEPROMADDR, 1, 8);
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

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
}

/* USER CODE BEGIN 4 */
////////////////////////////////////////////////////////////////////
#ifdef USE_SERIAL
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart1)
	{
		g_lineBuffer[sizeof(g_lineBuffer) - 1 - huart->RxXferCount] = 0;
		g_lineReceived = 1;
	}
}
#endif
////////////////////////////////////////////////////////////////////
void DisplayResults(uint8_t line)
{
	//TODO: optimize
#if defined(DEBUG_SERIAL) || defined(USE_LCD)
	uint8_t			irqEnabled = __get_PRIMASK() == 0;
	__disable_irq();
	DETECTORSTATUS	st = g_statuses[line];
	if(irqEnabled) __enable_irq();
#endif

#if defined(DEBUG_SERIAL)
	if(g_config.debug & (line+1))
	{
		// 1: | 20CDC000, 8336, 8337,         1,    10,        41,       1
		//ch st sum,      avg,  lastMeasured, diff, tolerance, threshold correction
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

#if defined(USE_LCD)
#if defined(DEBUG_LCD)
	I2cLcd_SetCursor(&g_lcd, 0, line);
	I2cLcd_PrintChar(&g_lcd, g_stateSyms[st.state]);
	I2cLcd_PrintChar(&g_lcd, ' ');
	I2cLcd_PrintInt(&g_lcd, st.diff, 0);
	I2cLcd_PrintStr(&g_lcd, "   ");
#else
	if(g_config.debug & 4) {
		uint32_t freq = ((uint32_t)((uint64_t)48000000*64 / st.lastMeasured));
		I2cLcd_SetCursor(&g_lcd, 0, line);
		I2cLcd_PrintUint(&g_lcd, freq, 0);
	}
#endif	//	DEBUG_LCD

#endif	//	USE_LCD

#if defined(USE_LCD) || defined(USE_LEDBAR)
	UpdateBar(line, CalcBar(&st));
#endif
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
