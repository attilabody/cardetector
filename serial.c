/*
 * serial.c
 *
 *  Created on: May 7, 2016
 *      Author: compi
 */
#include "config.h"
#include <avr/io.h>
#include <util/setbaud.h>

#include <stdlib.h>

//******************************************************************
void uart_init(void)
{
    UBRR0H = UBRRH_VALUE;
    UBRR0L = UBRRL_VALUE;

#if USE_2X
    UCSR0A |= _BV(U2X0);
#else
    UCSR0A &= ~(_BV(U2X0));
#endif

    UCSR0C = _BV(UCSZ01) | _BV(UCSZ00); /* 8-bit data */
    UCSR0B = _BV(RXEN0) | _BV(TXEN0);   /* Enable RX and TX */
}

//******************************************************************
// function to send data
void uart_transmit (unsigned char data)
{
    while (!(UCSR0A & (1<<UDRE0)));	// wait while register is free
    UDR0 = data;					// load data in the register
}

//******************************************************************
// function to receive data
unsigned char uart_recieve (void)
{
    while(!(UCSR0A & (1<<RXC0)));	// wait while data is being received
    return UDR0;					// return 8-bit data
}

//******************************************************************
void uart_print(char *str)
{
	while(*str) {
		uart_transmit(*str++);
	}
}

//******************************************************************
void uart_printlong(long l)
{
	char	buffer[sizeof(l)*8+1];
	ltoa(l, buffer, 10);
	uart_print(buffer);
}
