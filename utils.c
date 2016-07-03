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

//////////////////////////////////////////////////////////////////////////////
unsigned char iscommand( char **inptr, const char *cmd, unsigned char pgmspace )
{
	size_t n = 0;
	char x;
	if( pgmspace )
		while((x = pgm_read_byte( cmd++ )) && x == (*inptr)[n] )
			++n;
	else
		while((x = *cmd++) && x == (*inptr)[n] )
			++n;
	char y = (*inptr)[n];
	if( x || !(isspace(y) || ispunct(y) || y == '\n' || !y ))
		return 0;
	*inptr += n;
	while( **inptr && (isspace(**inptr)))
		++*inptr;
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

//////////////////////////////////////////////////////////////////////////////
char convertdigit( char c, uint8_t decimal )
{
	if( decimal )
		return (c >= '0' && c <= '9') ? c - '0' : -1;
	else {
		if( c >= '0' && c <= '9' ) return c - '0';
		if( c >= 'a' && c <= 'f' ) return c - 'a' + 10;
		if( c >= 'A' && c <= 'F' ) return c - 'A' + 10;
		return -1;
	}
}


//////////////////////////////////////////////////////////////////////////////
long getintparam( char **input, uint8_t decimal, uint8_t trimstart, uint8_t acceptneg )
{
	long	retval = 0;
	char	converted;
	uint8_t	found = 0;
	uint8_t	negative = 0;

	if( trimstart )
		while( **input && convertdigit( **input, decimal ) == -1 && **input != '-' )
			++*input;

	while( **input ) {
		if(( converted = convertdigit( **input, decimal )) == -1) {
			if( ! retval ) {
				if( **input == 'x' || **input == 'X' ) {
					decimal = 0;
					converted = 0;
				} else if( **input == '-' ) {
					negative = 1;
					converted = 0;
				} else break;
			} else break;
		}
		retval *=  decimal ? 10 : 16;
		retval += converted;
		found = 1;
		++*input;
	}
	return found ? (negative ? 0 - retval : retval ) : (acceptneg ? LONG_MIN : -1);
}


