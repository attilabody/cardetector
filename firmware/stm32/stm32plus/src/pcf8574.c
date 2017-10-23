/*
 * pcf8574.c
 *
 *  Created on: Mar 4, 2017
 *      Author: compi
 */
#include <stm32plus/pcf8574.h>
#include <stm32plus/i2cmaster.h>

#define I2C_RD I2cMaster_Read
#define I2C_WR I2cMaster_Write

void Pcf8574_Init(Pcf8574_Status *st, I2cMaster_State *i2cst, uint16_t i2cAddress)
{
	st->i2cStatus = i2cst;
	st->i2cAddress = i2cAddress;
	st->data = 0;
}

inline HAL_StatusTypeDef Pcf8574_Read(Pcf8574_Status * const st)
{
	return I2C_RD(st->i2cStatus, st->i2cAddress, &st->data, sizeof(st->data));
}

inline HAL_StatusTypeDef Pcf8574_Write(Pcf8574_Status * const st)
{
	return I2C_WR(st->i2cStatus, st->i2cAddress, &st->data, sizeof(st->data));
}

HAL_StatusTypeDef Pcf8574_WritePort(Pcf8574_Status *st, uint8_t value)
{
	st->data = value;
	return Pcf8574_Write(st);
}

HAL_StatusTypeDef Pcf8574_WritePin(Pcf8574_Status *st, uint8_t pin, uint8_t value)
{
	if(!value)
		st->data &= ~(1 << pin);
	else
		st->data |= (1 << pin);

	return Pcf8574_Write(st);
}

HAL_StatusTypeDef Pcf8574_ReadPort(Pcf8574_Status *st, uint8_t * const value)
{
	HAL_StatusTypeDef ret = Pcf8574_Read(st);
	*value = st->data;
	return ret;
}

HAL_StatusTypeDef Pcf8574_ReadPin(Pcf8574_Status *st, uint8_t pin, uint8_t * const value)
{
	HAL_StatusTypeDef ret = Pcf8574_Read(st);
	*value = (st->data & (1 << pin)) != 0;
	return ret;
}

HAL_StatusTypeDef Pcf8574_TogglePin(Pcf8574_Status * const st, uint8_t pin)
{
	st->data ^= (1 << pin);
	return Pcf8574_Write(st);
}

HAL_StatusTypeDef Pcf8574_ShiftRight(Pcf8574_Status * const st, uint8_t n)
{
	st->data >>= n;
	return Pcf8574_Write(st);
}

HAL_StatusTypeDef Pcf8574_ShiftLeft(Pcf8574_Status * const st, uint8_t n)
{
	st->data <<= n;
	return Pcf8574_Write(st);
}

