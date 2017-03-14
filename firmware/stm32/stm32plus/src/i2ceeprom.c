/*
 * i2ceeprom.c
 *
 *  Created on: Mar 11, 2017
 *      Author: compi
 */
#include <stm32plus/stm32_hal.h>
#include <stm32plus/i2ceeprom.h>

#define I2CMASTER_RD I2cMaster_ReadMem
#define I2CMASTER_WR I2cMaster_WriteMem

//////////////////////////////////////////////////////////////////////////////
static uint32_t I2cEEPROM_PollStatus(I2cEEPROM_State *st)
{
	uint32_t	ret;
	uint32_t	start = HAL_GetTick();

	while((ret = I2cMaster_Write(st->i2c, st->i2cAddress, NULL, 0)) != HAL_I2C_ERROR_NONE) {
		if(HAL_GetTick() - start >=10)
			return ret;
		HAL_Delay(1);
	}
	return ret;
}

//////////////////////////////////////////////////////////////////////////////
void I2cEEPROM_Init(I2cEEPROM_State *st, I2cMaster_State *i2c, uint16_t I2cAddress, uint8_t addressLengt, uint8_t pageLength)
{
	st->i2c = i2c;
	st->i2cAddress = I2cAddress;
	st->addressLengt = addressLengt;
	st->pageLength = pageLength;
	st->needPoll = 0;
}

//////////////////////////////////////////////////////////////////////////////
HAL_StatusTypeDef I2cEEPROM_Read(I2cEEPROM_State *st, uint32_t address, void* _buffer, uint32_t length)
{
	uint32_t	ret = HAL_OK;
	uint8_t		toRead;
	uint8_t		*buffer = (uint8_t*)_buffer;

	if(st->needPoll) {
		ret = I2cEEPROM_PollStatus(st);
		if(ret != HAL_OK)
			return ret;
	}

	toRead  = st->pageLength - (address & (st->pageLength -1));
	if(toRead > length)
		toRead = length;

	while(length && ret == HAL_OK)
	{
		ret = I2CMASTER_RD(st->i2c, st->i2cAddress, address, st->addressLengt, buffer, toRead);
		length -= toRead;
		buffer += toRead;
		address += toRead;
		toRead = length < st->pageLength ? length : st->pageLength;
	}
	return ret;
}

//////////////////////////////////////////////////////////////////////////////
HAL_StatusTypeDef I2cEEPROM_Write(I2cEEPROM_State *st, uint32_t address, const void* _buffer, uint32_t length)
{
	HAL_StatusTypeDef	ret = HAL_OK;
	uint8_t				toWrite;
	uint8_t				*buffer = (uint8_t*)_buffer;

	toWrite  = st->pageLength - (address & (st->pageLength -1));
	if(toWrite > length)
		toWrite = length;

	while(length && ret == HAL_OK)
	{
		if(st->needPoll) {
			ret = I2cEEPROM_PollStatus(st);
			if(ret != HAL_OK)
				return ret;
		}
		ret = I2CMASTER_WR(st->i2c, st->i2cAddress, address, st->addressLengt, buffer, toWrite);
		st->needPoll = 1;
		length -= toWrite;
		buffer += toWrite;
		address += toWrite;
		toWrite = length < st->pageLength ? length : st->pageLength;
	}
	return ret;
}

