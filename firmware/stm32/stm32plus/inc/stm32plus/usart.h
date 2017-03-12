#pragma once
#ifndef _CDC_USART_H_INCLUDED
#define _CDC_USART_H_INCLUDED

#include "stm32_hal.h"

////////////////////////////////////////////////////////////////////
extern UART_HandleTypeDef	*g_huart;

inline void __attribute__((always_inline)) UsartInit(UART_HandleTypeDef* huart) {	g_huart = huart; }
uint16_t	UsartSend(const void *buffer, uint16_t count, uint8_t block);
uint16_t	UsartSendStr(const char *buffer, uint8_t block);
uint16_t	UsartSendInt(int32_t data, uint8_t hex, uint8_t block);
uint16_t	UsartSendUint(uint32_t data, uint8_t hex, uint8_t block);

#endif	//	_USART_H_INCLUDED
