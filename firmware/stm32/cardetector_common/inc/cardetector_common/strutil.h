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
size_t uitodec(unsigned int data, char* buffer);
size_t uitohex(unsigned int data, char* buffer);

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
