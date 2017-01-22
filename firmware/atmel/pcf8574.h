//
//    FILE: PCF8574.H
//  AUTHOR: Rob Tillaart
//    DATE: 02-febr-2013
// VERSION: 0.1.04
// PURPOSE: I2C PCF8574 library for Arduino
//     URL:
//
// HISTORY:
// see PCF8574.cpp file
//

#if !defined(_PCF8574_H) && defined(HAVE_I2C) && defined(USE_I2C)
#define _PCF8574_H
#include <stdint.h>

typedef struct {
	uint8_t	address;
	uint8_t	data;
	uint8_t	error;
} PCF8574_STATUS;

void	pcf8574_init(PCF8574_STATUS *st, uint8_t address, uint8_t value);
uint8_t pcf8574_read8(PCF8574_STATUS *st, uint8_t *value);
uint8_t pcf8574_read(PCF8574_STATUS *st, uint8_t pin, uint8_t *value);

uint8_t pcf8574_write8(PCF8574_STATUS *st, uint8_t value);
uint8_t pcf8574_write(PCF8574_STATUS *st, uint8_t pin, uint8_t value);

uint8_t pcf8574_toggle(PCF8574_STATUS *st, uint8_t pin);
uint8_t pcf8574_shiftRight(PCF8574_STATUS *st, uint8_t n);
uint8_t pcf8574_shiftLeft(PCF8574_STATUS *st, uint8_t n);

uint8_t	pcf8574_read_st(PCF8574_STATUS *st);
uint8_t	pcf8574_write_st(PCF8574_STATUS *st);

inline uint8_t	pcf8574_lastval(PCF8574_STATUS *st) { return st->data; }
#endif
//
// END OF FILE
//
