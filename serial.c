/*
 * serial.c
 *
 *  Created on: May 7, 2016
 *      Author: compi
 */
#include "config.h"
#if (defined(DEBUG_TIMERS) || defined(DEBUG_DETECTOR) || defined(SRIAL_COMMANDS)) && defined(HAVE_SERIAL)
#include <avr/io.h>
#include <util/setbaud.h>
#include <avr/interrupt.h>
#include <string.h>

#include <stdlib.h>

#include "serial.h"

#define	RXBSMASK 0x0f
#define TXBSMASK 0x0f
volatile unsigned char	g_rxbuffer[RXBSMASK + 1];
volatile unsigned char	g_rxstart = 0, g_rxcount = 0, g_rxoverrun = 0;
volatile unsigned char	g_txbuffer[TXBSMASK + 1];
volatile unsigned char	g_txstart = 0, g_txcount = 0;


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
    UCSR0B = _BV(RXEN0) | _BV(TXEN0) | _BV(RXCIE0);   /* Enable RX and TX */
}

//******************************************************************
// function to send data
void uart_transmit_direct (unsigned char data)
{
    while (!(UCSR0A & _BV(UDRE0)));	// wait while register is free
    UDR0 = data;					// load data in the register
}

//******************************************************************
// function to receive data
unsigned char uart_recieve (void)
{
    while(!(UCSR0A & _BV(RXC0)));	// wait while data is being received
    return UDR0;					// return 8-bit data
}

////////////////////////////////////////////////////////////////////
void uart_printlong(long l)
{
	char	buffer[sizeof(l)*3+1];
	ltoa(l, buffer, 10);
	uart_send(buffer, strlen(buffer), 1);
}

////////////////////////////////////////////////////////////////////
int	uart_receive()
{
	unsigned char	tmp;

	if(!g_rxcount) return -1;
	cli();
	tmp = g_rxbuffer[g_rxstart++];
	g_rxstart &= RXBSMASK;
	--g_rxcount;
	sei();
	return tmp;
}

////////////////////////////////////////////////////////////////////
ISR(USART_RX_vect)
{
	unsigned char	r = UDR0;
	if(g_rxcount < sizeof(g_rxbuffer)) {
		g_rxbuffer[(g_rxstart + g_rxcount++) & RXBSMASK] = r;
	} else {
		g_rxoverrun = 1;
	}
}


////////////////////////////////////////////////////////////////////
size_t filltxbuffer(unsigned char *buffer, size_t count)
{
	size_t	copied = 0;
	while(g_txcount < sizeof(g_txbuffer) && count--) {
		g_txbuffer[(g_txstart + g_txcount++) & TXBSMASK] = *buffer++;
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
#if 1
ISR(USART_UDRE_vect)
{
	if(g_txcount) {
		UDR0 = g_txbuffer[g_txstart++];
		g_txstart &= TXBSMASK;
		--g_txcount;
	} else {
		UCSR0B &= ~_BV(UDRIE0);
	}
}
#endif

#endif	//	#if defined(DEBUG_TIMERS) || defined(DEBUG_DETECTOR)

