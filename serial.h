/*
 * serial.h
 *
 *  Created on: May 7, 2016
 *      Author: compi
 */

#if (defined(DEBUG_TIMERS) || defined(DEBUG_DETECTOR) || defined(DEBUG_LCD))&& defined(HAVE_SERIAL) && !defined(SERIAL_H_)
#define SERIAL_H_

#ifdef __cplusplus
extern "C" {
#endif	//	__cplusplus
#include "string.h"

void uart_init(void);
size_t	uart_send(void *buffer, size_t count, unsigned char block);
unsigned char uart_recieve (void);
inline void uart_print(const char *str) { uart_send((void*)str, strlen(str), 1); }
inline void uart_transmit (unsigned char data) { uart_send((void*)&data, 1, 1); }
inline void uart_println(const char *str) {
	uart_print(str);
	uart_transmit('\r');
	uart_transmit('\n');
}
void uart_printlong(long l);
int uart_receive();

#ifdef __cplusplus
}
#endif	//	__cplusplus

#endif /* SERIAL_H_ */
