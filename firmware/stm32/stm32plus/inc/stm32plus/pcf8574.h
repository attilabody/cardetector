/*
 * pcf8574.h
 *
 *  Created on: Mar 4, 2017
 *      Author: compi
 */
#pragma once
#ifndef CARDETECTOR_COMMON_INC_CARDETECTOR_COMMON_PCF8574_H_
#define CARDETECTOR_COMMON_INC_CARDETECTOR_COMMON_PCF8574_H_

#include <stm32plus/i2cmaster.h>

typedef struct _Pcf8574_Status
{
	I2cMaster_State 	*i2cStatus;
	uint16_t			i2cAddress;
	uint8_t				data;
} Pcf8574_Status;

void Pcf8574_Init(Pcf8574_Status *st, I2cMaster_State *i2cst, uint16_t i2cAddress);
HAL_StatusTypeDef Pcf8574_WritePort(Pcf8574_Status *st, uint8_t value);
HAL_StatusTypeDef Pcf8574_WritePin(Pcf8574_Status *st, uint8_t pin, uint8_t value);
HAL_StatusTypeDef Pcf8574_ReadPort(Pcf8574_Status *st, uint8_t * const value);
HAL_StatusTypeDef Pcf8574_ReadPin(Pcf8574_Status *st, uint8_t pin, uint8_t * const value);
HAL_StatusTypeDef Pcf8574_TogglePin(Pcf8574_Status * const st, uint8_t pin);
HAL_StatusTypeDef Pcf8574_ShiftRight(Pcf8574_Status * const st, uint8_t n);
HAL_StatusTypeDef Pcf8574_ShiftLeft(Pcf8574_Status * const st, uint8_t n);

#endif /* CARDETECTOR_COMMON_INC_CARDETECTOR_COMMON_PCF8574_H_ */
