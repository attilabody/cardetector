/*
 * i2cmaster.c
 *
 *  Created on: Mar 4, 2017
 *      Author: compi
 */
#include <stm32plus/stm32_hal.h>
#include <stm32plus/i2cmaster.h>

#ifndef COUNTOF
#define COUNTOF(x) (sizeof(x)/sizeof(x[0]))
#endif	//	COUNTOF

//////////////////////////////////////////////////////////////////////////////
typedef enum
{
	None,
	MasterTxCpltCallback,
	MasterRxCpltCallback,
	SlaveTxCpltCallback,
	SlaveRxCpltCallback,
	MemTxCpltCallback,
	MemRxCpltCallback,
	ErrorCallback
}  I2cMaster_CallbackType;

//////////////////////////////////////////////////////////////////////////////
 struct _I2cMaster_State {
	I2C_HandleTypeDef					*hi2c;
	volatile I2cMaster_CallbackType		expectedCallback;
	uint32_t 							callbackError;
};

I2cMaster_State	g_i2cStatuses[2];
uint8_t				g_numI2cStatuses = 0;

//////////////////////////////////////////////////////////////////////////////
inline I2cMaster_State *FindStatus(I2C_HandleTypeDef *hi2c)
{
	I2cMaster_State	*st = g_i2cStatuses;

	for(st = g_i2cStatuses; st < &g_i2cStatuses[g_numI2cStatuses]; ++st)
		if(hi2c == st->hi2c)
			return st;
	return NULL;
}

//////////////////////////////////////////////////////////////////////////////
uint8_t I2cMaster_Callback(I2C_HandleTypeDef *hi2c, I2cMaster_CallbackType type)
{
	I2cMaster_State	*st = FindStatus(hi2c);

	if(!st)
		return 0;

	if(st->expectedCallback == type || type == ErrorCallback ) {
		st->expectedCallback = None;
		st->callbackError = type == ErrorCallback ? HAL_I2C_GetError(hi2c) : HAL_I2C_ERROR_NONE;
	}
	return 1;
}

//////////////////////////////////////////////////////////////////////////////
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	I2cMaster_Callback(hi2c, MasterTxCpltCallback);
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	I2cMaster_Callback(hi2c, MasterRxCpltCallback);
}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	I2cMaster_Callback(hi2c, SlaveTxCpltCallback);
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	I2cMaster_Callback(hi2c, SlaveRxCpltCallback);
}

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	I2cMaster_Callback(hi2c, MemTxCpltCallback);
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	I2cMaster_Callback(hi2c, MemRxCpltCallback);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
	I2cMaster_Callback(hi2c, ErrorCallback);
}

//////////////////////////////////////////////////////////////////////////////
I2cMaster_State * I2cMaster_Init(I2C_HandleTypeDef *hi2c)
{
	if(g_numI2cStatuses == COUNTOF(g_i2cStatuses))
		return NULL;

	I2cMaster_State *st = &g_i2cStatuses[g_numI2cStatuses++];
	st->hi2c = hi2c;
	st->expectedCallback = None;
	st->callbackError = HAL_I2C_ERROR_NONE;

	return st;
}

//////////////////////////////////////////////////////////////////////////////
HAL_StatusTypeDef I2cMaster_WaitCallback(I2cMaster_State *st)
{
	while(st->expectedCallback != None) {}
	return st->callbackError ? HAL_ERROR : HAL_OK;
}

//////////////////////////////////////////////////////////////////////////////
uint32_t I2cMaster_GetCallbackError(I2cMaster_State *st)
{
	uint32_t ret = st->callbackError;
	st->callbackError = HAL_I2C_ERROR_NONE;
	return ret;
}

//////////////////////////////////////////////////////////////////////////////
uint32_t I2cMaster_GetI2cError(I2cMaster_State *st)
{
	return HAL_I2C_GetError(st->hi2c);
}

//////////////////////////////////////////////////////////////////////////////
HAL_StatusTypeDef I2cMaster_Write(I2cMaster_State *st, const uint16_t i2cAddress, const uint8_t *data, uint8_t size)
{
	if(I2cMaster_WaitCallback(st) != HAL_OK)
		return HAL_ERROR;

	return HAL_I2C_Master_Transmit(st->hi2c, i2cAddress, (uint8_t*)data, size, HAL_MAX_DELAY);
}

//////////////////////////////////////////////////////////////////////////////
HAL_StatusTypeDef I2cMaster_Write_IT(I2cMaster_State *st, const uint16_t i2cAddress, const uint8_t *data, uint8_t size)
{
	HAL_StatusTypeDef ret;

	if(I2cMaster_WaitCallback(st) != HAL_OK)
		return HAL_ERROR;

	ret = HAL_I2C_Master_Transmit_IT(st->hi2c, i2cAddress, (uint8_t*)data, size);
	if(ret == HAL_OK) {
		st->expectedCallback = MasterTxCpltCallback;
	}
	return ret;
}

//////////////////////////////////////////////////////////////////////////////
HAL_StatusTypeDef I2cMaster_Write_DMA(I2cMaster_State *st, const uint16_t i2cAddress, const uint8_t *data, uint8_t size)
{
	HAL_StatusTypeDef ret;

	if(I2cMaster_WaitCallback(st) != HAL_OK)
		return HAL_ERROR;

	ret = HAL_I2C_Master_Transmit_DMA(st->hi2c, i2cAddress, (uint8_t*)data, size);
	if(ret == HAL_OK) {
		st->expectedCallback = MasterTxCpltCallback;
	}
	return ret;
}

//////////////////////////////////////////////////////////////////////////////
HAL_StatusTypeDef I2cMaster_Read(I2cMaster_State *st, const uint16_t i2cAddress, uint8_t *data, uint8_t size)
{
	if(I2cMaster_WaitCallback(st) != HAL_OK)
		return HAL_ERROR;

	return HAL_I2C_Master_Receive(st->hi2c, i2cAddress, data, size, HAL_MAX_DELAY);
}

//////////////////////////////////////////////////////////////////////////////
HAL_StatusTypeDef I2cMaster_Read_IT(I2cMaster_State *st, const uint16_t i2cAddress, uint8_t *data, uint8_t size)
{
	HAL_StatusTypeDef ret;

	if(I2cMaster_WaitCallback(st) != HAL_OK)
		return HAL_ERROR;

	ret = HAL_I2C_Master_Receive_IT(st->hi2c, i2cAddress, data, size);
	if(ret == HAL_OK) {
		st->expectedCallback = MasterRxCpltCallback;
	}
	return ret;

}

//////////////////////////////////////////////////////////////////////////////
HAL_StatusTypeDef I2cMaster_Read_DMA(I2cMaster_State *st, const uint16_t i2cAddress, uint8_t *data, uint8_t size)
{
	HAL_StatusTypeDef ret;

	if(I2cMaster_WaitCallback(st) != HAL_OK)
		return HAL_ERROR;

	ret = HAL_I2C_Master_Receive_DMA(st->hi2c, i2cAddress, data, size);
	if(ret == HAL_OK) {
		st->expectedCallback = MasterRxCpltCallback;
	}
	return ret;
}

//////////////////////////////////////////////////////////////////////////////
HAL_StatusTypeDef I2cMaster_WriteMem(I2cMaster_State *st, const uint16_t i2cAddress, uint16_t memAddr, uint8_t memAddrSize, const uint8_t *data, uint16_t size)
{
	if(I2cMaster_WaitCallback(st) != HAL_OK)
		return HAL_ERROR;
	return HAL_I2C_Mem_Write(st->hi2c, i2cAddress, memAddr, memAddrSize, (uint8_t*)data, size, HAL_MAX_DELAY);
}

//////////////////////////////////////////////////////////////////////////////
HAL_StatusTypeDef I2cMaster_WriteMem_IT(I2cMaster_State *st, const uint16_t i2cAddress, uint16_t memAddr, uint8_t memAddrSize, const uint8_t *data, uint16_t size)
{
	HAL_StatusTypeDef ret;

	if(I2cMaster_WaitCallback(st) != HAL_OK)
		return HAL_ERROR;

	ret = HAL_I2C_Mem_Write_IT(st->hi2c, i2cAddress, memAddr, memAddrSize, (uint8_t*)data, size);
	if(ret == HAL_OK) {
		st->expectedCallback = MemTxCpltCallback;
	}
	return ret;
}

//////////////////////////////////////////////////////////////////////////////
HAL_StatusTypeDef I2cMaster_WriteMem_DMA(I2cMaster_State *st, const uint16_t i2cAddress, uint16_t memAddr, uint8_t memAddrSize, const uint8_t *data, uint16_t size)
{
	HAL_StatusTypeDef ret;

	if(I2cMaster_WaitCallback(st) != HAL_OK)
		return HAL_ERROR;

	ret = HAL_I2C_Mem_Write_DMA(st->hi2c, i2cAddress, memAddr, memAddrSize, (uint8_t*)data, size);
	if(ret == HAL_OK) {
		st->expectedCallback = MemTxCpltCallback;
	}
	return ret;
}

//////////////////////////////////////////////////////////////////////////////
HAL_StatusTypeDef I2cMaster_ReadMem(I2cMaster_State *st, const uint16_t i2cAddress, uint16_t memAddr, uint8_t memAddrSize, uint8_t *data, uint16_t size)
{
	if(I2cMaster_WaitCallback(st) != HAL_OK)
		return HAL_ERROR;

	return HAL_I2C_Mem_Read(st->hi2c, i2cAddress, memAddr, memAddrSize, data, size, HAL_MAX_DELAY);
}

//////////////////////////////////////////////////////////////////////////////
HAL_StatusTypeDef I2cMaster_ReadMem_IT(I2cMaster_State *st, const uint16_t i2cAddress, uint16_t memAddr, uint8_t memAddrSize, uint8_t *data, uint16_t size)
{
	HAL_StatusTypeDef ret;

	if(I2cMaster_WaitCallback(st) != HAL_OK)
		return HAL_ERROR;

	ret = HAL_I2C_Mem_Read_IT(st->hi2c, i2cAddress, memAddr, memAddrSize, data, size);
	if(ret == HAL_OK) {
		st->expectedCallback = MemRxCpltCallback;
	}
	return ret;
}

//////////////////////////////////////////////////////////////////////////////
HAL_StatusTypeDef I2cMaster_ReadMem_DMA(I2cMaster_State *st, const uint16_t i2cAddress, uint16_t memAddr, uint8_t memAddrSize, uint8_t *data, uint16_t size)
{
	HAL_StatusTypeDef ret;

	if(I2cMaster_WaitCallback(st) != HAL_OK)
		return HAL_ERROR;

	ret = HAL_I2C_Mem_Read_DMA(st->hi2c, i2cAddress, memAddr, memAddrSize, data, size);
	if(ret == HAL_OK) {
		st->expectedCallback = MemRxCpltCallback;
	}
	return ret;
}
