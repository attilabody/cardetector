/*
 * utils.h
 *
 *  Created on: Jun 26, 2016
 *      Author: compi
 */

#ifndef UTILS_H_
#define UTILS_H_
unsigned char getlinefromserial( unsigned char *buffer, unsigned char buflen, unsigned char *idx );
unsigned char iscommand( char **inptr, const char *cmd, unsigned char pgmspace );
long getintparam( char **input, uint8_t decimal, uint8_t trimstart, uint8_t acceptneg );

#endif /* UTILS_H_ */
