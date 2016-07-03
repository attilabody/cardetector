/*
 * serial.h
 *
 *  Created on: May 7, 2016
 *      Author: compi
 */

#ifndef SERIAL_H_
#define SERIAL_H_

#ifdef __cplusplus
extern "C" {
#endif	//	__cplusplus

#ifdef HAVE_SERIAL
#include <stddef.h>

extern volatile unsigned char g_rxcount;

#ifndef SERIAL_RX_BUFFER_SIZE
#define SERIAL_RX_BUFFER_SIZE 16
#endif	//	SERIAL_RX_BUFFER_SIZE
#ifndef SERIAL_TX_BUFFER_SIZE
#define SERIAL_TX_BUFFER_SIZE 16
#endif	//	SERIAL_TX_BUFFER_SIZE

void uart_init(unsigned long baud_rate);
size_t	uart_send(void *buffer, size_t count, unsigned char block);
unsigned char uart_recieve (void);
void uart_printstr(const char *str);
void uart_printstr_p(const char *str);
static inline void uart_printchar (unsigned char data) { uart_send((void*)&data, 1, 1); }
static inline void uart_println(const char *str) {
	uart_printstr(str);
	uart_printstr("\r\n");
}
static inline unsigned char uart_available() { return g_rxcount != 0; }
void uart_printlong(long l);
int uart_receive();

unsigned char getlinefromserial( unsigned char *buffer, unsigned char buflen, unsigned char *idx );

#else	//	HAVE_SERIAL
static inline void uart_init(unsigned long baud_rate) {}
static inline size_t uart_send(void *buffer, size_t count, unsigned char block) {return count;}
static inline unsigned char uart_recieve (void) {return 0;}
static inline void uart_printstr(const char *str) {}
static inline void uart_printstr_p(const char *str) {}
static inline void uart_printchar (unsigned char data) {}
static inline void uart_println(const char *str) {}
static inline unsigned char uart_available() { return 0; }
static inline void uart_printlong(long l) {}
static inline int uart_receive() {return -1;}

static inline unsigned char getlinefromserial( unsigned char *buffer, unsigned char buflen, unsigned char *idx ) {return 0;}
#endif	//	else of HAVE_SERIAL

#ifdef __cplusplus
}
#endif	//	__cplusplus

#endif /* SERIAL_H_ */
