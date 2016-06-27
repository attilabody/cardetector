#include "config.h"
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>

#include <stdlib.h>
#include <string.h>
#include <limits.h>

#include "serial.h"
#include "utils.h"
#include "commsyms.h"

unsigned char 	g_linebuffer[128];
unsigned char	g_lineidx;

////////////////////////////////////////////////////////////////////
int main(void)
{
	uart_init(BAUD);

    sei();

    while(1)
	{
    	while(!getlinefromserial(g_linebuffer, sizeof(g_linebuffer), &g_lineidx));
    	uart_printchar(CMNT);
    	uart_send(g_linebuffer, g_lineidx, 1);
    	g_lineidx = 0;
	}
	return 0;
}

////////////////////////////////////////////////////////////////////

