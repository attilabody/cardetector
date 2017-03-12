/*
 * i2cmaster.h
 *
 *  Created on: Mar 4, 2017
 *      Author: compi
 */

#pragma once
#ifndef CARDETECTOR_COMMON_INC_CARDETECTOR_COMMON_I2CMASTER_H_
#define CARDETECTOR_COMMON_INC_CARDETECTOR_COMMON_I2CMASTER_H_

#include <cardetector_common/stm32_hal.h>

struct _I2cMaster_State;
typedef struct _I2cMaster_State I2cMaster_State;

typedef enum { Poll, It, Dma } I2cMaster_Mode;
I2cMaster_State * I2cMaster_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef I2cMaster_Write(I2cMaster_State *st, const uint16_t i2cAddress, const uint8_t *data, uint8_t size, I2cMaster_Mode mode);
HAL_StatusTypeDef I2cMaster_Read(I2cMaster_State *st, const uint16_t i2cAddress, uint8_t *data, uint8_t size, I2cMaster_Mode mode);
HAL_StatusTypeDef I2cMaster_WriteMem(I2cMaster_State *st, const uint16_t i2cAddress, uint16_t memAddr, uint8_t memAddrSize, const uint8_t *data, uint16_t size, I2cMaster_Mode mode);
HAL_StatusTypeDef I2cMaster_ReadMem(I2cMaster_State *st, const uint16_t i2cAddress, uint16_t memAddr, uint8_t memAddrSize, uint8_t *data, uint16_t size, I2cMaster_Mode mode);

HAL_StatusTypeDef	I2cMaster_WaitCallback(I2cMaster_State *st);
uint32_t			I2cMaster_GetCallbackError(I2cMaster_State *st);
uint32_t			I2cMaster_GetI2cError(I2cMaster_State *st);

#endif /* CARDETECTOR_COMMON_INC_CARDETECTOR_COMMON_I2CMASTER_H_ */