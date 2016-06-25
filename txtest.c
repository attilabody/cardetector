#include "config.h"
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>

#define BAUD 115200

#include <util/setbaud.h>

#include <stdlib.h>
#include <string.h>
#include <limits.h>

#include "serial.h"

char 	txstr[] = "lofaszbingo\r\n";
uint8_t	txidx = 0;
uint8_t	txstrlen = 0;

//#define OWN_ISR

////////////////////////////////////////////////////////////////////
int main(void)
{
	uart_init();

    txstrlen =  strlen(txstr);

    sei();

    while(1)
	{
    	uart_send(txstr, txstrlen, 1);
    	_delay_ms(100);
	}
	return 0;
}

////////////////////////////////////////////////////////////////////
#ifdef OWN_ISR
ISR(USART_UDRE_vect)
{
	UDR0 = txstr[txidx++];
	if(txidx == txstrlen) {
		txidx = 0;
		UCSR0B &= ~_BV(UDRIE0);
	}
}
#endif
