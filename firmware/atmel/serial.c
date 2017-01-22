/*
 * serial.c
 *
 *  Created on: May 7, 2016
 *      Author: compi
 */
#include "config.h"
#if defined(HAVE_SERIAL)
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

#include <string.h>
#include <stdlib.h>

#include "serial.h"

volatile unsigned char	g_rxbuffer[SERIAL_RX_BUFFER_SIZE];
volatile unsigned char	g_rxstart = 0, g_rxcount = 0, g_rxoverrun = 0;
volatile unsigned char	g_txbuffer[SERIAL_TX_BUFFER_SIZE];
volatile unsigned char	g_txstart = 0, g_txcount = 0;


//******************************************************************
void uart_init(unsigned long baud_rate)
{
	unsigned int prescaler = (F_CPU / 4 / baud_rate - 1) / 2;
	UCSR0A = 1 << U2X0;

	if(((F_CPU == 16000000UL) && (baud_rate == 57600)) || (prescaler > 4095)) {
		UCSR0A = 0;
		prescaler = (F_CPU / 8 / baud_rate - 1) / 2;
	}
	UBRR0H = prescaler >> 8;
	UBRR0L = prescaler;

    UCSR0C = _BV(UCSZ01) | _BV(UCSZ00);				// N81
    UCSR0B = _BV(RXEN0) | _BV(TXEN0) | _BV(RXCIE0);	// Enable RX and TX
}

////////////////////////////////////////////////////////////////////
void uart_printlong(long l)
{
	char	buffer[sizeof(l)*3+1];
	ltoa(l, buffer, 10);
	uart_send(buffer, strlen(buffer), 1);
}


////////////////////////////////////////////////////////////////////
void uart_printstr(const char *str)
{
	uart_send((void*)str, strlen(str), 1);
}


////////////////////////////////////////////////////////////////////
void uart_printstr_p(const char *str)
{
	char c;
	while((c = pgm_read_byte(str++)))
		uart_send(&c, 1, 1);
}


////////////////////////////////////////////////////////////////////
int	uart_receive()
{
	unsigned char	tmp;

	if(!g_rxcount) return -1;
	cli();
	tmp = g_rxbuffer[g_rxstart++];
	if(g_rxstart == SERIAL_RX_BUFFER_SIZE)
		g_rxstart = 0;
	--g_rxcount;
	sei();
	return tmp;
}


////////////////////////////////////////////////////////////////////
ISR(USART_RX_vect)
{
	unsigned char	r = UDR0;
	if(g_rxcount < sizeof(g_rxbuffer)) {
		unsigned char pos = g_rxstart + g_rxcount++;
		if(pos >= SERIAL_RX_BUFFER_SIZE)
			pos -= SERIAL_RX_BUFFER_SIZE;
		g_rxbuffer[pos] = r;
	} else {
		g_rxoverrun = 1;
	}
}


////////////////////////////////////////////////////////////////////
static size_t filltxbuffer(unsigned char *buffer, size_t count)
{
	size_t	copied = 0;
	while(g_txcount < sizeof(g_txbuffer) && count--) {
		unsigned char pos = g_txstart + g_txcount++;
		if(pos >= SERIAL_RX_BUFFER_SIZE)
			pos -= SERIAL_RX_BUFFER_SIZE;
		g_txbuffer[pos] = *buffer++;
		++copied;
	}
	return copied;
}


////////////////////////////////////////////////////////////////////
size_t	uart_send(void *buffer, size_t count, unsigned char block)
{
	size_t	sent = 0, copied;
	while(count)
	{
		while(g_txcount == sizeof(g_txbuffer))
			if( !block) return sent;
		cli();
		copied = filltxbuffer(buffer, count);
		if(!(UCSR0B & _BV(UDRIE0))) UCSR0B |= _BV(UDRIE0);
		sei();
		buffer = (unsigned char*)buffer + copied;
		count -= copied;
		sent += copied;
	}
	return sent;
}

////////////////////////////////////////////////////////////////////
ISR(USART_UDRE_vect)
{
	if(g_txcount) {
		UDR0 = g_txbuffer[g_txstart++];
		if(g_txstart == SERIAL_TX_BUFFER_SIZE )
		g_txstart = 0;
		--g_txcount;
	} else {
		UCSR0B &= ~_BV(UDRIE0);
	}
}


//////////////////////////////////////////////////////////////////////////////
unsigned char getlinefromserial( unsigned char *buffer, unsigned char buflen, unsigned char *idx )
{
#if defined(DEBUG_SERIALIN)
	if(uart_available())
	{
		uart_printstr(CMNTS "called with buflen ");
		uart_printlong(buflen);
		uart_printstr(" and idx ");
		uart_printlong(*idx);
	}
#endif	//	DEBUG_SERIALIN
	unsigned char lineready = 0;
	while(uart_available() && !lineready ) {
		char inc = uart_receive();
#if defined(DEBUG_SERIALIN)
		buffer[*idx ] = 0;
		uart_printchar(CMNT);
		uart_printstr( (char*)buffer );
		uart_printchar(' ');
		uart_printchar(inc);
		uart_printchar(' ');
		uart_printlong( *idx );
		uart_printstr("\r\n");
#endif	//	DEBUG_SERIALIN
		if( inc == '\r' ) continue;
		if( inc == '\n' ) inc = 0;
		buffer[(*idx)++] = inc;
		if( !inc || *idx >= buflen - 1 ) {
			if( inc )
				buffer[*idx] = 0;
			lineready = 1;
#if defined(DEBUG_SERIALIN)
			uart_printstr(CMNTS "Line ready: ");
			uart_printstr( (char*)buffer );
			uart_printchar('|');
			uart_printlong(inc);
			uart_printchar(' ');
			uart_printlong(*idx);
			uart_printchar(CMNT);
			for( unsigned char _idx = 0; _idx < *idx; ++_idx ) {
				uart_printlong(buffer[_idx]);
				uart_printchar(' ');
			}
			uart_printstr("\r\n");
#endif	//	DEBUG_SERIALIN
		}
	}
	return lineready;
}

#endif	//	#if defined(HAVE_SERIAL)

