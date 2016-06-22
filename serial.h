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

void uart_init(void);
void uart_transmit (unsigned char data);
unsigned char uart_recieve (void);
void uart_print(const char *str);
void uart_println(const char *str) {
	uart_print(str);
	uart_transmit('\r');
	uart_transmit('\n');
}
void uart_printlong(long l);

#ifdef __cplusplus
}
#endif	//	__cplusplus

#endif /* SERIAL_H_ */
