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

typedef struct _I2cLcd_State
{
	I2cMaster_State	*m_i2c;
	uint8_t				m_data;
	uint16_t			m_i2cAddress;
} I2cLcd_State;

void I2cLcd_Init(I2cLcd_State *st, I2cMaster_State *i2cst, uint16_t i2cAddress);
HAL_StatusTypeDef I2cLcd_InitDisplay(I2cLcd_State *st );
HAL_StatusTypeDef I2cLcd_Clear(I2cLcd_State *st);
HAL_StatusTypeDef I2cLcd_Home(I2cLcd_State *st);
HAL_StatusTypeDef I2cLcd_SetCursor(I2cLcd_State *st, uint8_t x, uint8_t y);
HAL_StatusTypeDef I2cLcd_PrintStr(I2cLcd_State *st, const char *str);
uint32_t I2cLcd_PrintChar(I2cLcd_State *st, const char c);
size_t I2cLcd_PrintInt(I2cLcd_State *st, int32_t i, uint8_t hex);
size_t I2cLcd_PrintUint(I2cLcd_State *st, uint32_t u, uint8_t hex);

#endif /* CARDETECTOR_COMMON_INC_CARDETECTOR_COMMON_I2CLCD_H_ */
