/*
  I2C.cpp - I2C library
  Copyright (c) 2011-2012 Wayne Truchsess.  All right reserved.
  Rev 5.0 - January 24th, 2012
          - Removed the use of interrupts completely from the library
            so TWI state changes are now polled.
          - Added calls to lockup() function in most functions
            to combat arbitration problems
          - Fixed scan() procedure which left timeouts enabled
            and set to 80msec after exiting procedure
          - Changed scan() address range back to 0 - 0x7F
          - Removed all Wire legacy functions from library
          - A big thanks to Richard Baldwin for all the testing
            and feedback with debugging bus lockups!
  Rev 4.0 - January 14th, 2012
          - Updated to make compatible with 8MHz clock frequency
  Rev 3.0 - January 9th, 2012
          - Modified library to be compatible with Arduino 1.0
          - Changed argument type from boolean to uint8_t in pullUp(),
            setSpeed() and receiveByte() functions for 1.0 compatability
          - Modified return values for timeout feature to report
            back where in the transmission the timeout occured.
          - added function scan() to perform a bus scan to find devices
            attached to the I2C bus.  Similar to work done by Todbot
            and Nick Gammon
  Rev 2.0 - September 19th, 2011
          - Added support for timeout function to prevent
            and recover from bus lockup (thanks to PaulS
            and CrossRoads on the Arduino forum)
          - Changed return type for stop() from void to
            uint8_t to handle timeOut function
  Rev 1.0 - August 8th, 2011

  This is a modified version of the Arduino Wire/TWI
  library.  Functions were rewritten to provide more functionality
  and also the use of Repeated Start.  Some I2C devices will not
  function correctly without the use of a Repeated Start.  The
  initial version of this library only supports the Master.


  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/


#include "i2c.h"
#include <inttypes.h>
#include <string.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

uint32_t	millis();

uint8_t i2c_start();
uint8_t i2c_sendAddress(uint8_t);
uint8_t i2c_sendByte(uint8_t);
uint8_t i2c_receiveByte(uint8_t);
uint8_t i2c_stop();
void	i2c_lockUp();

static uint8_t	g_i2c_returnStatus;
static uint8_t	g_i2c_nack;
static uint8_t	g_i2c_data[MAX_BUFFER_SIZE];
static uint8_t	g_i2c_bytesAvailable = 0;
static uint8_t	g_i2c_bufferIndex = 0;
static uint8_t	g_i2c_totalBytes = 0;
static uint16_t	g_i2c_timeOutDelay = 0;

#define HANDLERETURNSTATUS(rs,rrs) \
	g_i2c_returnStatus=rs;\
	if( g_i2c_returnStatus ) {\
		if( g_i2c_returnStatus == 1 )\
			return rrs; \
		return g_i2c_returnStatus; \
	}

#define HANDLERETURNSTATUSEXCEPT(rs,exc,rrs) \
	g_i2c_returnStatus=rs;\
	if( g_i2c_returnStatus == 1 )\
		return rrs; \
	if( g_i2c_returnStatus != exc)	return g_i2c_returnStatus; \

////////////// Public Methods ////////////////////////////////////////



void i2c_begin()
{
#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega8__) || defined(__AVR_ATmega328P__)
	// activate internal pull-ups for twi
	// as per note from atmega8 manual pg167
	sbi(PORTC, 4);
	sbi(PORTC, 5);
#else
	// activate internal pull-ups for twi
	// as per note from atmega128 manual pg204
	sbi(PORTD, 0);
	sbi(PORTD, 1);
#endif
	// initialize twi prescaler and bit rate
	cbi(TWSR, TWPS0);
	cbi(TWSR, TWPS1);
	TWBR = ((F_CPU / 100000) - 16) / 2;
	// enable twi module and acks
	TWCR = _BV(TWEN) | _BV(TWEA);
}

void i2c_end()
{
	TWCR = 0;
}

void i2c_timeOut(uint16_t _timeOut)
{
	g_i2c_timeOutDelay = _timeOut;
}

void i2c_setSpeed(uint8_t _fast)
{
	if(!_fast) {
		TWBR = ((F_CPU / 100000) - 16) / 2;
	} else {
		TWBR = ((F_CPU / 400000) - 16) / 2;
	}
}

void i2c_pullup(uint8_t activate)
{
  if(activate)
  {
    #if defined(__AVR_ATmega168__) || defined(__AVR_ATmega8__) || defined(__AVR_ATmega328P__)
      // activate internal pull-ups for twi
      // as per note from atmega8 manual pg167
      sbi(PORTC, 4);
      sbi(PORTC, 5);
    #else
      // activate internal pull-ups for twi
      // as per note from atmega128 manual pg204
      sbi(PORTD, 0);
      sbi(PORTD, 1);
    #endif
  }
  else
  {
    #if defined(__AVR_ATmega168__) || defined(__AVR_ATmega8__) || defined(__AVR_ATmega328P__)
      // deactivate internal pull-ups for twi
      // as per note from atmega8 manual pg167
      cbi(PORTC, 4);
      cbi(PORTC, 5);
    #else
      // deactivate internal pull-ups for twi
      // as per note from atmega128 manual pg204
      cbi(PORTD, 0);
      cbi(PORTD, 1);
    #endif
  }
}

//void i2c_scan()
//{
//	uint16_t tempTime = g_i2c_timeOutDelay;
//	timeOut(80);
//	uint8_t totalDevicesFound = 0;
//	Serial.println("Scanning for devices...please wait");
//	Serial.println();
//	for(uint8_t s = 0; s <= 0x7F; s++)
//	{
//		g_i2c_returnStatus = 0;
//		g_i2c_returnStatus = start();
//		if(!g_i2c_returnStatus)
//		{
//			g_i2c_returnStatus = sendAddress(SLA_W(s));
//		}
//		if(g_i2c_returnStatus)
//		{
//			if(g_i2c_returnStatus == 1)
//			{
//				Serial.println("There is a problem with the bus, could not complete scan");
//				g_i2c_timeOutDelay = tempTime;
//				return;
//			}
//		}
//		else
//		{
//			Serial.print("Found device at address - ");
//			Serial.print(" 0x");
//			Serial.println(s,HEX);
//			totalDevicesFound++;
//		}
//		stop();
//	}
//	if(!totalDevicesFound){Serial.println("No devices found");}
//	g_i2c_timeOutDelay = tempTime;
//}


uint8_t i2c_available()
{
  return(g_i2c_bytesAvailable);
}

uint8_t i2c_receive()
{
  g_i2c_bufferIndex = g_i2c_totalBytes - g_i2c_bytesAvailable;
  if(!g_i2c_bytesAvailable)  {
    g_i2c_bufferIndex = 0;
    return(0);
  }
  g_i2c_bytesAvailable--;
  return(g_i2c_data[g_i2c_bufferIndex]);
}


/*return values for new functions that use the timeOut feature
  will now return at what point in the transmission the timeout
  occurred. Looking at a full communication sequence between a
  master and slave (transmit data and then readback data) there
  a total of 7 points in the sequence where a timeout can occur.
  These are listed below and correspond to the returned value:
  1 - Waiting for successful completion of a Start bit
  2 - Waiting for ACK/NACK while addressing slave in transmit mode (MT)
  3 - Waiting for ACK/NACK while sending data to the slave
  4 - Waiting for successful completion of a Repeated Start
  5 - Waiting for ACK/NACK while addressing slave in receiver mode (MR)
  6 - Waiting for ACK/NACK while receiving data from the slave
  7 - Waiting for successful completion of the Stop bit

  All possible return values:
  0           Function executed with no errors
  1 - 7       Timeout occurred, see above list
  8 - 0xFF    See datasheet for exact meaning */


/////////////////////////////////////////////////////

uint8_t i2c_write(uint8_t address, uint8_t data)
{
	g_i2c_returnStatus = 0;
	g_i2c_returnStatus = i2c_start();
	if (g_i2c_returnStatus)
	{
		return (g_i2c_returnStatus);
	}
	HANDLERETURNSTATUS(i2c_sendAddress(SLA_W(address)), 2);
	HANDLERETURNSTATUS(i2c_sendByte(data), 3);
	HANDLERETURNSTATUS(i2c_stop(), 7);
	return (g_i2c_returnStatus);
}

uint8_t i2c_writereg(uint8_t address, uint8_t registerAddress, uint8_t data)
{
	g_i2c_returnStatus = 0;
	g_i2c_returnStatus = i2c_start();
	if(g_i2c_returnStatus) {return(g_i2c_returnStatus);}
	HANDLERETURNSTATUS(i2c_sendAddress(SLA_W(address)), 2);
	HANDLERETURNSTATUS(i2c_sendByte(registerAddress), 3);
	HANDLERETURNSTATUS(i2c_sendByte(data), 3);
	HANDLERETURNSTATUS(i2c_stop(), 7);
	return(g_i2c_returnStatus);
}

uint8_t i2c_writereg16(uint8_t address, uint16_t registerAddress, uint8_t data)
{
	g_i2c_returnStatus = 0;
	g_i2c_returnStatus = i2c_start();
	if (g_i2c_returnStatus) return (g_i2c_returnStatus);
	HANDLERETURNSTATUS(i2c_sendAddress(SLA_W(address)), 2);
	HANDLERETURNSTATUS(i2c_sendByte(registerAddress >> 8), 3);
	HANDLERETURNSTATUS(i2c_sendByte(registerAddress & 0xff), 3);
	HANDLERETURNSTATUS(i2c_sendByte(data), 3);
	HANDLERETURNSTATUS(i2c_stop(), 7);
	return(g_i2c_returnStatus);
}

uint8_t i2c_writestr(uint8_t address, uint8_t registerAddress, char *data)
{
  uint8_t bufferLength = strlen(data);
  g_i2c_returnStatus = 0;
  g_i2c_returnStatus = i2c_writebuf(address, registerAddress, (uint8_t*)data, bufferLength);
  return(g_i2c_returnStatus);
}

uint8_t i2c_writestr16(uint8_t address, uint16_t registerAddress, char *data)
{
  uint8_t bufferLength = strlen(data);
  g_i2c_returnStatus = 0;
  g_i2c_returnStatus = i2c_writebuf16(address, registerAddress, (uint8_t*)data, bufferLength);
  return(g_i2c_returnStatus);
}

uint8_t i2c_writebuf(uint8_t address, uint8_t registerAddress, uint8_t *data, uint8_t count)
{
	g_i2c_returnStatus = 0;
	g_i2c_returnStatus = i2c_start();
	if(g_i2c_returnStatus){return(g_i2c_returnStatus);}
	HANDLERETURNSTATUS(i2c_sendAddress(SLA_W(address)), 2);
	HANDLERETURNSTATUS(i2c_sendByte(registerAddress), 3);
	for (uint8_t i = 0; i < count; i++) {
		HANDLERETURNSTATUS(i2c_sendByte(data[i]), 3);
	}
	HANDLERETURNSTATUS(i2c_stop(), 7);
	return(g_i2c_returnStatus);
}

uint8_t i2c_writebuf16(uint8_t address, uint16_t registerAddress, uint8_t *data, uint8_t count)
{
	g_i2c_returnStatus = 0;
	g_i2c_returnStatus = i2c_start();
	if(g_i2c_returnStatus){return(g_i2c_returnStatus);}
	HANDLERETURNSTATUS(i2c_sendAddress(SLA_W(address)), 2);
	HANDLERETURNSTATUS(i2c_sendByte(registerAddress >> 8), 3);
	HANDLERETURNSTATUS(i2c_sendByte(registerAddress & 0xff), 3);
	for (uint8_t i = 0; i < count; i++) {
		HANDLERETURNSTATUS(i2c_sendByte(data[i]), 3);
	}
	HANDLERETURNSTATUS(i2c_stop(), 7);
	return(g_i2c_returnStatus);
}

uint8_t i2c_read(uint8_t address, uint8_t count)
{
	g_i2c_bytesAvailable = 0;
	g_i2c_bufferIndex = 0;
	if(count == 0){count++;}
	g_i2c_nack = count - 1;
	g_i2c_returnStatus = 0;
	g_i2c_returnStatus = i2c_start();
	if(g_i2c_returnStatus){return(g_i2c_returnStatus);}
	HANDLERETURNSTATUS(i2c_sendAddress(SLA_R(address)), 5);
	for(uint8_t i = 0; i < count; i++)
	{
		if( i == g_i2c_nack ) {
			HANDLERETURNSTATUSEXCEPT(i2c_receiveByte(0), MR_DATA_NACK, 6);
		} else {
			HANDLERETURNSTATUSEXCEPT(i2c_receiveByte(1), MR_DATA_ACK, 6);
		}
		g_i2c_data[i] = TWDR;
		g_i2c_bytesAvailable = i+1;
		g_i2c_totalBytes = i+1;
	}
	HANDLERETURNSTATUS(i2c_stop(), 7);
	return(g_i2c_returnStatus);
}

uint8_t i2c_readreg(uint8_t address, uint8_t registerAddress, uint8_t count)
{
	g_i2c_bytesAvailable = 0;
	g_i2c_bufferIndex = 0;
	if(count == 0) { count++; }
	g_i2c_nack = count - 1;
	g_i2c_returnStatus = 0;
	g_i2c_returnStatus = i2c_start();
	if(g_i2c_returnStatus) { return (g_i2c_returnStatus); }
	HANDLERETURNSTATUS(i2c_sendAddress(SLA_W(address)), 2);
	HANDLERETURNSTATUS(i2c_sendByte(registerAddress), 3);
	HANDLERETURNSTATUS(i2c_start(), 4);
	HANDLERETURNSTATUS(i2c_sendAddress(SLA_R(address)), 5);
	for (uint8_t i = 0; i < count; i++) {
		if( i == g_i2c_nack ) {
			HANDLERETURNSTATUSEXCEPT(i2c_receiveByte(0), MR_DATA_NACK, 6);
		} else {
			HANDLERETURNSTATUSEXCEPT(i2c_receiveByte(1), MR_DATA_ACK, 6);
		}
		g_i2c_data[i] = TWDR;
		g_i2c_bytesAvailable = i + 1;
		g_i2c_totalBytes = i + 1;
	}
	HANDLERETURNSTATUS(i2c_stop(), 7);
	return (g_i2c_returnStatus);
}

uint8_t i2c_readreg16(uint8_t address, uint16_t registerAddress, uint8_t count)
{
	g_i2c_bytesAvailable = 0;
	g_i2c_bufferIndex = 0;
	if(count == 0){count++;}
	g_i2c_nack = count - 1;

	g_i2c_returnStatus = i2c_start();
	if(g_i2c_returnStatus){return(g_i2c_returnStatus);}
	HANDLERETURNSTATUS(i2c_sendAddress(SLA_W(address)), 2);
	HANDLERETURNSTATUS(i2c_sendByte(registerAddress >> 8), 3);
	HANDLERETURNSTATUS(i2c_sendByte(registerAddress & 0xff), 3);
	HANDLERETURNSTATUS(i2c_start(), 4);
	HANDLERETURNSTATUS(i2c_sendAddress(SLA_R(address)), 5);
	for(uint8_t i = 0; i < count; i++)
	{
		if( i == g_i2c_nack ) {
			g_i2c_returnStatus = i2c_receiveByte(0);
			if(g_i2c_returnStatus == 1){return(6);}
			if(g_i2c_returnStatus != MR_DATA_NACK){return(g_i2c_returnStatus);}
		} else {
			g_i2c_returnStatus = i2c_receiveByte(1);
			if(g_i2c_returnStatus == 1){return(6);}
			if(g_i2c_returnStatus != MR_DATA_ACK){return(g_i2c_returnStatus);}
		}
		g_i2c_data[i] = TWDR;
		g_i2c_bytesAvailable = i+1;
		g_i2c_totalBytes = i+1;
	}
	HANDLERETURNSTATUS(i2c_stop(), 7);
	return(g_i2c_returnStatus);
}

uint8_t i2c_readbuf(uint8_t address, uint8_t count, uint8_t *dataBuffer)
{
	g_i2c_bytesAvailable = 0;
	g_i2c_bufferIndex = 0;
	if(count == 0){count++;}
	g_i2c_nack = count - 1;
	g_i2c_returnStatus = 0;
	g_i2c_returnStatus = i2c_start();
	if(g_i2c_returnStatus){return(g_i2c_returnStatus);}
	HANDLERETURNSTATUS(i2c_sendAddress(SLA_R(address)), 5);
	for(uint8_t i = 0; i < count; i++)
	{
		if( i == g_i2c_nack ) {
			g_i2c_returnStatus = i2c_receiveByte(0);
			if(g_i2c_returnStatus == 1){return(6);}
			if(g_i2c_returnStatus != MR_DATA_NACK){return(g_i2c_returnStatus);}
		} else {
			g_i2c_returnStatus = i2c_receiveByte(1);
			if(g_i2c_returnStatus == 1){return(6);}
			if(g_i2c_returnStatus != MR_DATA_ACK){return(g_i2c_returnStatus);}
		}
		dataBuffer[i] = TWDR;
		g_i2c_bytesAvailable = i+1;
		g_i2c_totalBytes = i+1;
	}
	HANDLERETURNSTATUS(i2c_stop(), 7);
	return(g_i2c_returnStatus);
}

uint8_t i2c_readbuf16(uint8_t address, uint16_t registerAddress, uint8_t count, uint8_t *dataBuffer)
{
	g_i2c_bytesAvailable = 0;
	g_i2c_bufferIndex = 0;
	if(count == 0){count++;}
	g_i2c_nack = count - 1;
	g_i2c_returnStatus = i2c_start();
	if(g_i2c_returnStatus){return(g_i2c_returnStatus);}
	HANDLERETURNSTATUS(i2c_sendAddress(SLA_W(address)), 2);
	HANDLERETURNSTATUS(i2c_sendByte(registerAddress >> 8), 3);
	HANDLERETURNSTATUS(i2c_sendByte(registerAddress & 0xff), 3);
	HANDLERETURNSTATUS(i2c_start(), 4);
	HANDLERETURNSTATUS(i2c_sendAddress(SLA_R(address)), 5);
	for(uint8_t i = 0; i < count; i++)
	{
		if( i == g_i2c_nack ) {
			g_i2c_returnStatus = i2c_receiveByte(0);
			if(g_i2c_returnStatus == 1){return(6);}
			if(g_i2c_returnStatus != MR_DATA_NACK){return(g_i2c_returnStatus);}
		} else {
			g_i2c_returnStatus = i2c_receiveByte(1);
			if(g_i2c_returnStatus == 1){return(6);}
			if(g_i2c_returnStatus != MR_DATA_ACK){return(g_i2c_returnStatus);}
		}
		dataBuffer[i] = TWDR;
		g_i2c_bytesAvailable = i+1;
		g_i2c_totalBytes = i+1;
	}
	HANDLERETURNSTATUS(i2c_stop(), 7);
	return(g_i2c_returnStatus);
}


uint8_t i2c_readbufreg(uint8_t address, uint8_t registerAddress, uint8_t count, uint8_t *dataBuffer)
{
	g_i2c_bytesAvailable = 0;
	g_i2c_bufferIndex = 0;
	if(count == 0){count++;}
	g_i2c_nack = count - 1;
	g_i2c_returnStatus = 0;
	g_i2c_returnStatus = i2c_start();
	if(g_i2c_returnStatus){return(g_i2c_returnStatus);}
	HANDLERETURNSTATUS(i2c_sendAddress(SLA_W(address)), 2);
	HANDLERETURNSTATUS(i2c_sendByte(registerAddress), 3);
	HANDLERETURNSTATUS(i2c_start(), 4);
	HANDLERETURNSTATUS(i2c_sendAddress(SLA_R(address)), 5);
	for(uint8_t i = 0; i < count; i++)
	{
		if( i == g_i2c_nack )
		{
			g_i2c_returnStatus = i2c_receiveByte(0);
			if(g_i2c_returnStatus == 1){return(6);}
			if(g_i2c_returnStatus != MR_DATA_NACK){return(g_i2c_returnStatus);}
		}
		else
		{
			g_i2c_returnStatus = i2c_receiveByte(1);
			if(g_i2c_returnStatus == 1){return(6);}
			if(g_i2c_returnStatus != MR_DATA_ACK){return(g_i2c_returnStatus);}
		}
		dataBuffer[i] = TWDR;
		g_i2c_bytesAvailable = i+1;
		g_i2c_totalBytes = i+1;
	}
	HANDLERETURNSTATUS(i2c_stop(), 6);
	return(g_i2c_returnStatus);
}


/////////////// Private Methods ////////////////////////////////////////


uint8_t i2c_start()
{
	unsigned long startingTime = millis();
	TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);

	while (!(TWCR & (1<<TWINT))) {
		if(!g_i2c_timeOutDelay){continue;}
		if((millis() - startingTime) >= g_i2c_timeOutDelay) {
			i2c_lockUp();
			return(1);
		}
	}

	if ((TWI_STATUS == START) || (TWI_STATUS == REPEATED_START)) {
		return(0);
	}

	if (TWI_STATUS == LOST_ARBTRTN) {
		uint8_t bufferedStatus = TWI_STATUS;
		i2c_lockUp();
		return(bufferedStatus);
	}

	return(TWI_STATUS);
}

uint8_t i2c_sendAddress(uint8_t i2cAddress)
{
	TWDR = i2cAddress;
	unsigned long startingTime = millis();
	TWCR = (1<<TWINT) | (1<<TWEN);

	while (!(TWCR & (1<<TWINT))) {
		if(!g_i2c_timeOutDelay){continue;}
		if((millis() - startingTime) >= g_i2c_timeOutDelay) {
			i2c_lockUp();
			return(1);
		}

	}

	if ((TWI_STATUS == MT_SLA_ACK) || (TWI_STATUS == MR_SLA_ACK)) {
		return(0);
	}

	uint8_t bufferedStatus = TWI_STATUS;

	if ((TWI_STATUS == MT_SLA_NACK) || (TWI_STATUS == MR_SLA_NACK)) {
		i2c_stop();
		return(bufferedStatus);
	} else {
		i2c_lockUp();
		return(bufferedStatus);
	}
}

uint8_t i2c_sendByte(uint8_t i2cData)
{
	TWDR = i2cData;
	unsigned long startingTime = millis();
	TWCR = (1<<TWINT) | (1<<TWEN);
	while (!(TWCR & (1<<TWINT))) {
		if(!g_i2c_timeOutDelay){continue;}
		if((millis() - startingTime) >= g_i2c_timeOutDelay) {
			i2c_lockUp();
			return(1);
		}
	}

	if (TWI_STATUS == MT_DATA_ACK) {
		return(0);
	}

	uint8_t bufferedStatus = TWI_STATUS;

	if (TWI_STATUS == MT_DATA_NACK) {
		i2c_stop();
		return(bufferedStatus);
	} else {
		i2c_lockUp();
		return(bufferedStatus);
	}
}

uint8_t i2c_receiveByte(uint8_t ack)
{
	unsigned long startingTime = millis();
	if(ack) {
		TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
	} else {
		TWCR = (1<<TWINT) | (1<<TWEN);
	}

	while (!(TWCR & (1<<TWINT)))
	{
		if(!g_i2c_timeOutDelay){continue;}
		if((millis() - startingTime) >= g_i2c_timeOutDelay) {
			i2c_lockUp();
			return(1);
		}
	}

	if (TWI_STATUS == LOST_ARBTRTN) {
		uint8_t bufferedStatus = TWI_STATUS;
		i2c_lockUp();
		return(bufferedStatus);
	}

	return(TWI_STATUS);
}

uint8_t i2c_stop()
{
	unsigned long startingTime = millis();
	TWCR = (1<<TWINT)|(1<<TWEN)| (1<<TWSTO);
	while ((TWCR & (1<<TWSTO))) {
		if(!g_i2c_timeOutDelay){continue;}
		if((millis() - startingTime) >= g_i2c_timeOutDelay) {
			i2c_lockUp();
			return(1);
		}
	}
	return(0);
}

void i2c_lockUp()
{
	TWCR = 0; //releases SDA and SCL lines to high impedance
	TWCR = _BV(TWEN) | _BV(TWEA); //reinitialize TWI
}

