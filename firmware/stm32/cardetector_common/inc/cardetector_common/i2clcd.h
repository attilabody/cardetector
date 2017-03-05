/*
 * i2clcd.h
 *
 *  Created on: Mar 5, 2017
 *      Author: compi
 */

#ifndef CARDETECTOR_COMMON_INC_CARDETECTOR_COMMON_I2CLCD_H_
#define CARDETECTOR_COMMON_INC_CARDETECTOR_COMMON_I2CLCD_H_

#include <cardetector_common/stm32_hal.h>
#include <cardetector_common/i2cmaster.h>

typedef struct _I2cLcd_Status
{
	I2cMaster_Status	*m_i2c;
	uint8_t				m_data;
	uint16_t			m_i2cAddress;
} I2cLcd_Status;

void I2cLcd_Init(I2cLcd_Status *st, I2cMaster_Status *i2cst, uint16_t i2cAddress);
HAL_StatusTypeDef I2cLcd_InitDisplay(I2cLcd_Status *st );
HAL_StatusTypeDef I2cLcd_Clear(I2cLcd_Status *st);
HAL_StatusTypeDef I2cLcd_Home(I2cLcd_Status *st);
HAL_StatusTypeDef I2cLcd_SetCursor(I2cLcd_Status *st, uint8_t x, uint8_t y);
HAL_StatusTypeDef I2cLcd_PrintStr(I2cLcd_Status *st, const char *str);
size_t I2cLcd_PrintInt(I2cLcd_Status *st, unsigned int u, uint8_t hex);

#endif /* CARDETECTOR_COMMON_INC_CARDETECTOR_COMMON_I2CLCD_H_ */
