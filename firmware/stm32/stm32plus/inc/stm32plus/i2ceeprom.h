/*
 * i2ceeprom.h
 *
 *  Created on: Mar 11, 2017
 *      Author: compi
 */

#ifndef CARDETECTOR_COMMON_INC_CARDETECTOR_COMMON_I2CEEPROM_H_
#define CARDETECTOR_COMMON_INC_CARDETECTOR_COMMON_I2CEEPROM_H_

#include <stm32plus/stm32_hal.h>
#include <stm32plus/i2cmaster.h>

typedef struct _I2cEEPROM_State
{
	I2cMaster_State	*i2c;
	uint16_t			i2cAddress;
	uint8_t				addressLengt;
	uint8_t				pageLength;
	uint8_t				needPoll;
} I2cEEPROM_State;

void I2cEEPROM_Init(I2cEEPROM_State *st, I2cMaster_State *i2c, uint16_t I2cAddress, uint8_t addressLengt, uint8_t pageLength);
HAL_StatusTypeDef I2cEEPROM_Read(I2cEEPROM_State *st, uint32_t address, void* _buffer, uint32_t length);
HAL_StatusTypeDef I2cEEPROM_Write(I2cEEPROM_State *st, uint32_t address, const void* _buffer, uint32_t length);

#endif /* CARDETECTOR_COMMON_INC_CARDETECTOR_COMMON_I2CEEPROM_H_ */
