/*
  I2C.h   - I2C library
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

#ifndef I2C_H_
#define I2C_H_

#include <inttypes.h>


#define START           0x08
#define REPEATED_START  0x10
#define MT_SLA_ACK		0x18
#define MT_SLA_NACK		0x20
#define MT_DATA_ACK     0x28
#define MT_DATA_NACK    0x30
#define MR_SLA_ACK		0x40
#define MR_SLA_NACK		0x48
#define MR_DATA_ACK     0x50
#define MR_DATA_NACK    0x58
#define LOST_ARBTRTN    0x38
#define TWI_STATUS      (TWSR & 0xF8)
#define SLA_W(address)  (address << 1)
#define SLA_R(address)  ((address << 1) + 0x01)
#define cbi(sfr, bit)   (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit)   (_SFR_BYTE(sfr) |= _BV(bit))

#define MAX_BUFFER_SIZE 32
#ifdef __cplusplus
extern "C" {
#endif	//	__cplusplus

void i2c_begin();
void i2c_end();
void i2c_timeOut(uint16_t);
void i2c_setSpeed(uint8_t);
void i2c_pullup(uint8_t);
void i2c_scan();
uint8_t i2c_available();
uint8_t i2c_receive();
uint8_t i2c_write(uint8_t address, uint8_t data);
uint8_t i2c_writereg(uint8_t address, uint8_t registerAddress, uint8_t data);
uint8_t i2c_writereg16(uint8_t address, uint16_t registerAddress, uint8_t data);
uint8_t i2c_writestr(uint8_t address, uint8_t registerAddress, char* data);
uint8_t i2c_writestr16(uint8_t address, uint16_t registerAddress, char* data);
uint8_t i2c_writebuf(uint8_t address, uint8_t registerAddress, uint8_t* data, uint8_t count);
uint8_t i2c_writebuf16(uint8_t address, uint16_t registerAddress, uint8_t* data, uint8_t count);
uint8_t i2c_read(uint8_t address, uint8_t count);
uint8_t i2c_readreg(uint8_t address, uint8_t registerAddress, uint8_t count);
uint8_t i2c_readreg16(uint8_t address, uint16_t registerAddress, uint8_t count);
uint8_t i2c_readbuf(uint8_t address, uint8_t count, uint8_t* dataBuffer);
uint8_t i2c_readbufreg(uint8_t address, uint8_t registeAddress, uint8_t count, uint8_t* dataBuffer);
uint8_t i2c_readbufreg16(uint8_t address, uint16_t registeAddress, uint8_t count, uint8_t* dataBuffer);

#ifdef __cplusplus
}
#endif	//	__cplusplus

#endif
