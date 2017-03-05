#pragma once
#ifndef _CDC_USART_H_INCLUDED
#define _CDC_USART_H_INCLUDED

#include "stm32_hal.h"

////////////////////////////////////////////////////////////////////
extern UART_HandleTypeDef	*g_huart;

inline void UsartInit(UART_HandleTypeDef* huart) {	g_huart = huart; }
uint16_t	UsartSend(const void *buffer, uint16_t count, uint8_t block);

#endif	//	_USART_H_INCLUDED
