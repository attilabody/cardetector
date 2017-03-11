/*
 * i2cmaster.c
 *
 *  Created on: Mar 4, 2017
 *      Author: compi
 */
#include <cardetector_common/stm32_hal.h>
#include <cardetector_common/i2cmaster.h>

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
 struct _I2cMaster_Status {
	I2C_HandleTypeDef					*m_hi2c;
	volatile I2cMaster_CallbackType		m_expectedCallback;
	uint32_t 							m_callbackError;
};

I2cMaster_Status	g_i2cStatuses[2];
uint8_t				g_numI2cStatuses = 0;

//////////////////////////////////////////////////////////////////////////////
inline I2cMaster_Status *FindStatus(I2C_HandleTypeDef *hi2c)
{
	I2cMaster_Status	*st = g_i2cStatuses;

	for(st = g_i2cStatuses; st < &g_i2cStatuses[g_numI2cStatuses]; ++st)
		if(hi2c == st->m_hi2c)
			return st;
	return NULL;
}

//////////////////////////////////////////////////////////////////////////////
uint8_t I2cMaster_Callback(I2C_HandleTypeDef *hi2c, I2cMaster_CallbackType type)
{
	I2cMaster_Status	*st = FindStatus(hi2c);

	if(!st)
		return 0;

	if(st->m_expectedCallback == type || type == ErrorCallback ) {
		st->m_expectedCallback = None;
		st->m_callbackError = type == ErrorCallback ? HAL_I2C_GetError(hi2c) : HAL_I2C_ERROR_NONE;
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
I2cMaster_Status * I2cMaster_Init(I2C_HandleTypeDef *hi2c)
{
	if(g_numI2cStatuses == COUNTOF(g_i2cStatuses))
		return NULL;

	I2cMaster_Status *st = &g_i2cStatuses[g_numI2cStatuses++];
	st->m_hi2c = hi2c;
	st->m_expectedCallback = None;
	st->m_callbackError = HAL_I2C_ERROR_NONE;

	return st;
}

//////////////////////////////////////////////////////////////////////////////
inline uint32_t WaitCallback(I2cMaster_Status *st)
{
	while(st->m_expectedCallback != None) {}
	return st->m_callbackError;
}

//////////////////////////////////////////////////////////////////////////////
HAL_StatusTypeDef I2cMaster_Write(I2cMaster_Status *st, const uint16_t i2cAddress, uint8_t *data, uint8_t size, I2cMaster_Mode mode)
{
	if(WaitCallback(st) != HAL_I2C_ERROR_NONE)
		return HAL_ERROR;
	if(mode == Poll) {
		return HAL_I2C_Master_Transmit(st->m_hi2c, i2cAddress, data, size, HAL_MAX_DELAY);
	} else {
		HAL_StatusTypeDef ret;
		if(mode == It) {
			ret = HAL_I2C_Master_Transmit_IT(st->m_hi2c, i2cAddress, data, size);
		} else {
			ret = HAL_I2C_Master_Transmit_DMA(st->m_hi2c, i2cAddress, data, size);
		}
		if(ret == HAL_OK) {
			st->m_expectedCallback = MasterTxCpltCallback;
		}
		return ret;
	}
}

//////////////////////////////////////////////////////////////////////////////
HAL_StatusTypeDef I2cMaster_Read(I2cMaster_Status *st, const uint16_t i2cAddress, uint8_t *data, uint8_t size, I2cMaster_Mode mode)
{
	if(WaitCallback(st) != HAL_I2C_ERROR_NONE)
		return HAL_ERROR;
	if(mode == Poll) {
		return HAL_I2C_Master_Receive(st->m_hi2c, i2cAddress, data, size, HAL_MAX_DELAY);
	} else {
		HAL_StatusTypeDef ret;
		if(mode == It) {
			ret = HAL_I2C_Master_Receive_IT(st->m_hi2c, i2cAddress, data, size);
		} else {
			ret = HAL_I2C_Master_Receive_DMA(st->m_hi2c, i2cAddress, data, size);
		}
		if(ret == HAL_OK) {
			st->m_expectedCallback = MasterRxCpltCallback;
		}
		return ret;
	}
}

//////////////////////////////////////////////////////////////////////////////
HAL_StatusTypeDef I2cMaster_WriteMem(I2cMaster_Status *st, const uint16_t i2cAddress, uint16_t memAddr, uint8_t memAddrSize, uint8_t *data, uint16_t size, I2cMaster_Mode mode)
{
	if(WaitCallback(st) != HAL_I2C_ERROR_NONE)
		return HAL_ERROR;
	if(mode == Poll) {
		return HAL_I2C_Mem_Write(st->m_hi2c, i2cAddress, memAddr, memAddrSize, data, size, HAL_MAX_DELAY);
	} else {
		HAL_StatusTypeDef ret;
		if(mode == It) {
			ret = HAL_I2C_Mem_Write_IT(st->m_hi2c, i2cAddress, memAddr, memAddrSize, data, size);
		} else {
			ret = HAL_I2C_Mem_Write_DMA(st->m_hi2c, i2cAddress, memAddr, memAddrSize, data, size);
		}
		if(ret == HAL_OK) {
			st->m_expectedCallback = MemTxCpltCallback;
		}
		return ret;
	}
}

//////////////////////////////////////////////////////////////////////////////
HAL_StatusTypeDef I2cMaster_ReadMem(I2cMaster_Status *st, const uint16_t i2cAddress, uint16_t memAddr, uint8_t memAddrSize, uint8_t *data, uint16_t size, I2cMaster_Mode mode)
{
	if(WaitCallback(st) != HAL_I2C_ERROR_NONE)
		return HAL_ERROR;
	if(mode == Poll) {
		return HAL_I2C_Mem_Read(st->m_hi2c, i2cAddress, memAddr, memAddrSize, data, size, HAL_MAX_DELAY);
	} else {
		HAL_StatusTypeDef ret;
		if(mode == It) {
			ret = HAL_I2C_Mem_Read_IT(st->m_hi2c, i2cAddress, memAddr, memAddrSize, data, size);
		} else {
			ret = HAL_I2C_Mem_Read_DMA(st->m_hi2c, i2cAddress, memAddr, memAddrSize, data, size);
		}
		if(ret == HAL_OK) {
			st->m_expectedCallback = MemRxCpltCallback;
		}
		return ret;
	}
}


