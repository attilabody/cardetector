#pragma once
#ifndef _STM32PLUS_USART_H
#define _STM32PLUS_USART_H

#include "stm32_hal.h"

////////////////////////////////////////////////////////////////////
#ifdef __cplusplus
extern "C" {
#endif

void 		UsartInit(UART_HandleTypeDef* huart);
uint16_t	UsartSend(const void *buffer, uint16_t count, uint8_t block);
uint16_t	UsartSendStr(const char *buffer, uint8_t block);
uint16_t	UsartPrintInt(int32_t data, uint8_t hex, uint8_t block);
uint16_t	UsartPrintUint(uint32_t data, uint8_t hex, uint8_t block);
uint16_t	UsartPrintByte(uint8_t data, uint8_t hex, uint8_t block);

#ifdef __cplusplus
}
#endif

#endif	/*	_STM32PLUS_USART_H */
