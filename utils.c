/*
 * utils.c
 *
 *  Created on: Jun 26, 2016
 *      Author: compi
 */

#include "config.h"
#include <avr/pgmspace.h>
#include <ctype.h>
#include <string.h>
#include "serial.h"
#include "utils.h"
#include "commsyms.h"

//#define DEBUG_SERIALIN
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

//////////////////////////////////////////////////////////////////////////////
unsigned char iscommand( char **inptr, const char *cmd, unsigned char pgmspace )
{
	size_t n = 0;
	char x;
	if( pgmspace )
		while((x = pgm_read_byte( cmd++ )) && x == *inptr[n] )
			++n;
	else
		while((x = *cmd++) && x == *inptr[n] )
			++n;
	char y = *inptr[n];
	if( x || !(isspace(y) || ispunct(y) || y == '\n' || !y ))
		return 0;
	inptr += n;
	while( **inptr && (isspace(**inptr)))
		++inptr;
	return 1;
}

////////////////////////////////////////////////////////////////////////////////
//char findcommand( const char **inptr, const char **commands )
//{
//	int		ret = 0;
//	while( **inptr && (isspace(**inptr) || ispunct(**inptr)) )
//		++*inptr;
//	if( !**inptr ) return -1;
//
//	while( *commands )
//	{
//		int cmdlen = 0;
//		while( (*inptr)[cmdlen] && !isspace((*inptr)[cmdlen]))
//			++cmdlen;
//		if (!strncmp(*inptr, *commands, cmdlen))
//		{
//			*inptr += cmdlen;
//			while( **inptr && (isspace(**inptr)))
//				++*inptr;
//			return ret;
//		}
//		++ret;
//		++commands;
//	}
//	return -1;
//}
//
