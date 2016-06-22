//
//    FILE: PCF8574.cpp
//  AUTHOR: Rob Tillaart
//    DATE: 02-febr-2013
// VERSION: 0.1.04
// PURPOSE: I2C PCF8574 library for Arduino
//     URL:
//
// HISTORY:
// 0.1.04 2015-05-09 removed ambiguity in read8()
// 0.1.03 2015-03-02 address int -> uint8_t
// 0.1.02 replaced ints with uint8_t to reduce footprint;
//        added default value for shiftLeft() and shiftRight()
//        renamed status() to lastError();
// 0.1.01 added value(); returns last read 8 bit value (cached);
//        value() does not always reflect the latest state of the pins!
// 0.1.00 initial version
//
#include "config.h"
#if defined(HAVE_I2C) && defined(USE_I2C)
#include "pcf8574.h"

#include "i2c.h"

uint8_t	pcf8574_read_st(PCF8574_STATUS *st)
{
	st->error = i2c_read(st->address, 1);
	if(!st->error) {
		st->data = i2c_receive();
	}
	return st->error;
}

uint8_t	pcf8574_write_st(PCF8574_STATUS *st)
{
	return (st->error = i2c_write(st->address, st->data));
}

void	pcf8574_init(PCF8574_STATUS *st, uint8_t address, uint8_t value)
{
	st->address = address;
	st->error = 0;
	st->data = value;
}

uint8_t pcf8574_write8(PCF8574_STATUS *st, uint8_t value)
{
	st->data = value;
    return pcf8574_write_st(st);
}

// pin should be 0..7
uint8_t pcf8574_read(PCF8574_STATUS *st, uint8_t pin, uint8_t *value)
{
    if(!pcf8574_read_st(st)) {
    	*value = (st->data & (1<<pin)) > 0;
    }
    return st->error;
}

// pin should be 0..7
uint8_t pcf8574_write(PCF8574_STATUS *st, uint8_t pin, uint8_t value)
{
#ifdef __PCF8574_VERBOSE
	Serial.print(F("pcf8574_write: "));
	Serial.print(pin);
	Serial.print(F(", "));
	Serial.println(value );
#endif	//	PCF8574_VERBOSE
	if (!value) st->data &= ~(1<<pin);
	else st->data |= (1<<pin);
	return pcf8574_write8(st, st->data);
}

// pin should be 0..7
uint8_t pcf8574_toggle(PCF8574_STATUS *st, uint8_t pin)
{
	st->data ^=  (1 << pin);
	st->error = pcf8574_write8(st, st->data);
	return st->error;
}

// n should be 0..7
uint8_t pcf8574_shiftRight(PCF8574_STATUS *st, uint8_t n)
{
	if (n == 0 || n > 7 ) return 0xff;

	st->data >>= n;
	return pcf8574_write8(st, st->data);
}

// n should be 0..7
uint8_t pcf8574_shiftLeft(PCF8574_STATUS *st, uint8_t n)
{
    if (n == 0 || n > 7) return 0xff;
	st->data <<= n;
	return pcf8574_write_st(st);
}

uint8_t pcf8574_read8(PCF8574_STATUS *st, uint8_t* value)
{
	pcf8574_read_st(st);
	if (!st->error) {
		*value = st->data;
	}
	return st->error;
}

#endif	//	#if defined(HAVE_I2C) && defined(USE_I2C)
//
// END OF FILE
//
