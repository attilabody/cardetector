/*
 * pcf8574.c
 *
 *  Created on: Mar 4, 2017
 *      Author: compi
 */
#include <stm32plus/pcf8574.h>
#include <stm32plus/i2cmaster.h>

void Pcf8574_Init(Pcf8574_Status *st, I2cMaster_State *i2cst, uint16_t i2cAddress)
{
	st->m_i2cStatus = i2cst;
	st->m_i2cAddress = i2cAddress;
	st->m_data = 0;
}

inline HAL_StatusTypeDef Pcf8574_Read(Pcf8574_Status * const st)
{
	return I2cMaster_Read_IT(st->m_i2cStatus, st->m_i2cAddress, &st->m_data, sizeof(st->m_data));
}

inline HAL_StatusTypeDef Pcf8574_Write(Pcf8574_Status * const st)
{
	return I2cMaster_Write_IT(st->m_i2cStatus, st->m_i2cAddress, &st->m_data, sizeof(st->m_data));
}

HAL_StatusTypeDef Pcf8574_WritePort(Pcf8574_Status *st, uint8_t value)
{
	st->m_data = value;
	return Pcf8574_Write(st);
}

HAL_StatusTypeDef Pcf8574_WritePin(Pcf8574_Status *st, uint8_t pin, uint8_t value)
{
	if(!value)
		st->m_data &= ~(1 << pin);
	else
		st->m_data |= (1 << pin);

	return Pcf8574_Write(st);
}

HAL_StatusTypeDef Pcf8574_ReadPort(Pcf8574_Status *st, uint8_t * const value)
{
	HAL_StatusTypeDef ret = Pcf8574_Read(st);
	*value = st->m_data;
	return ret;
}

HAL_StatusTypeDef Pcf8574_ReadPin(Pcf8574_Status *st, uint8_t pin, uint8_t * const value)
{
	HAL_StatusTypeDef ret = Pcf8574_Read(st);
	*value = (st->m_data & (1 << pin)) != 0;
	return ret;
}

HAL_StatusTypeDef Pcf8574_TogglePin(Pcf8574_Status * const st, uint8_t pin)
{
	st->m_data ^= (1 << pin);
	return Pcf8574_Write(st);
}

HAL_StatusTypeDef Pcf8574_ShiftRight(Pcf8574_Status * const st, uint8_t n)
{
	st->m_data >>= n;
	return Pcf8574_Write(st);
}

HAL_StatusTypeDef Pcf8574_ShiftLeft(Pcf8574_Status * const st, uint8_t n)
{
	st->m_data <<= n;
	return Pcf8574_Write(st);
}

