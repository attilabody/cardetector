/*
 * strutil.h
 *
 *  Created on: Feb 11, 2017
 *      Author: compi
 */

#ifndef APPLICATION_USER_STRUTIL_H_
#define APPLICATION_USER_STRUTIL_H_

#include <stddef.h>
#include <inttypes.h>

//////////////////////////////////////////////////////////////////////////////
size_t uitodec(char* buffer, uint32_t data);
size_t uitohex(char* buffer, uint32_t data, uint8_t chars);
size_t itodec(char* buffer, int data);
size_t itohex(char* buffer, int data);

//////////////////////////////////////////////////////////////////////////////
inline void strrev(char *first, char *last)
{
	char tmp;
	while(last > first) {
		tmp = *first;
		*first++ = *last;
		*last-- = tmp;
	}
}

//////////////////////////////////////////////////////////////////////////////
inline char tochr(const uint8_t in, const uint8_t upper)
{
	return in + ((in < 10) ? '0' : (upper ? 'A' : 'a') - 10);
}


#endif /* APPLICATION_USER_STRUTIL_H_ */
