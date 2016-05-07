/*
 * serial.h
 *
 *  Created on: May 7, 2016
 *      Author: compi
 */

#ifndef SERIAL_H_
#define SERIAL_H_

void uart_init(void);
void uart_transmit (unsigned char data);
unsigned char uart_recieve (void);
void uart_print(char *str);
void uart_println(char *str) {
	uart_print(str);
	uart_transmit('\n');
}
void uart_printlong(long l);

#endif /* SERIAL_H_ */
