/*
 * i2c.c
 *
 *  Created on: Oct 23, 2019
 *      Author: Al-Shimaa
 */


#include "i2c.h"
uint8 status = 0;
void TWI_init()
{
	TWBR = 0x34;
	TWSR = 0x00;
	/*
	 *  No prescaler and TWBR = 0x34 with FCPU = 12MHZ >>> bit rate = 100,000 kps.
	 *
	 */

	TWAR = 0b00000010;
	/* Sets this MC as 0x0000001
	 * TWGCE = 0 >> General call is off
	 *
	 */
	TWCR = (1<<TWEN);
		/*
		 *  TWEN = 1 >> enable TWI module
		 *  TWie= 0 >> disable interrupts.
		 */
}

void TWI_start()
{
	TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
	/*
	 * TWINT = 1  >> Clear the flag before sending start bit.
	 * TWISTA = 1 >> Send the start bit
	 * TWEN = 1 >> Enables module
	 */
	while(BIT_IS_CLEAR(TWCR,TWINT)) {} // wait until start bit is send already.

}

void TWI_stop()
{
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
	/*
		 * TWINT = 1  >> Clear the flag before sending start bit.
		 * TWISTA = 1 >> Send the stop bit
		 * TWEN = 1 >> Enables module
		 */
	while(BIT_IS_CLEAR(TWCR,TWSTO)) {} // wait until stop bit is send already.


}

void TWI_write(uint8 data)
{
	TWDR = data; // put data into the register
	TWCR = (1 << TWINT) | (1 << TWEN); // clear the flag and enable the module.
	while (BIT_IS_CLEAR(TWCR,TWINT)) {}// wait until data sent successfully.

}

uint8 TWI_read_ack(void)
{
	TWCR = (1<<TWINT) | (1<< TWEN) | (1<<TWEA); // clear flag and enable module and enable sending ACK
	while(BIT_IS_CLEAR(TWCR,TWINT)) {}
	return TWDR;
}

uint8 TWI_read_nack(void)
{
	TWCR = (1<<TWINT) | (1<< TWEN); // clear flag and enable module
	while(BIT_IS_CLEAR(TWCR,TWINT)) {}
	return TWDR;
}

uint8 TWI_getStatus(void)
{
	uint8 status = TWSR & 0xF8;
	return (status);
}

/* new functions test */

// function to send byte to external device register //
/*
 * devAddr >> device address
 * regAddr >> register address
 * byte >> 8 bit data to send
 */
void TWI_send_byte(uint8 devAddr,uint8 regAddr,uint8 byte)
{
	uint8 address = (devAddr<<1);      // 7 bit address with bit 0 - > r/w = 0 to write.
	TWI_start();					// send start bit
	TWI_write(address);				// send device address
	TWI_write(regAddr);				// send register address
	TWI_write(byte);				// send the byte
	TWI_stop();						// send stop bit

}

void TWI_send_byte_no_stop_bit(uint8 devAddr,uint8 regAddr,uint8 byte)
{
	uint8 address = (devAddr<<1);      // 7 bit address with bit 0 - > r/w = 0 to write.
	TWI_start();					// send start bit
	TWI_write(address);				// send device address
	TWI_write(regAddr);				// send register address
	TWI_write(byte);				// send the byte

}


// function to read byte from external device register
/*
 *  devAddr >> device address
 *  regAddr >> register address
 *  data >> 8bit variable to store the recieved byte
 */

uint8 TWI_recieve_byte_nack(uint8 devAddr,uint8 regAddr, uint8* data)
{

	uint8 address = (devAddr<<1);		// 7 bit address with WRITE -> r/w = 0;
	TWI_start();
	if (TWI_getStatus() != TW_START)
	        return 10;
	TWI_write(address);					// sends device address
	 if (TWI_getStatus() != TW_MT_SLA_W_ACK)
	        return 11;
	TWI_write(regAddr);					// sends register address
	if (TWI_getStatus() != TW_MT_DATA_ACK)
	        return 12;
	TWI_start();						// send repeated start bit
	if (TWI_getStatus() != TW_REP_START)
	        return 13;
	address = (devAddr<<1) | (1<<0);	// 7bit address with READ -> r/w = 1

	TWI_write(address);					// send device address again
	if (TWI_getStatus() != TW_MT_SLA_R_ACK)
		        return 14;
	*data = TWI_read_nack();		// read the byte and save in data.
	if (TWI_getStatus() != TW_MR_DATA_NACK)
	        return 15;
	TWI_stop();
	return 1;
}

void TWI_recieve_byte_with_ack(uint8 devAddr,uint8 regAddr, uint8* data) // not stop byte send! , u need to send it urself
{
	uint8 address = (devAddr<<1);		// 7 bit address with WRITE -> r/w = 0;
	TWI_start();
	TWI_write(address);					// sends device address
	TWI_write(regAddr);					// sends register address
	TWI_start();						// send repeated start bit
	address = (devAddr<<1) | (1<<0);	// 7bit address with READ -> r/w = 1
	TWI_write(address);					// send device address again
	*data = TWI_read_ack();		// read the byte and save in data.
}


// function to write specfic bits in a register //
/*
 *  devAddr >> device address
 *  regAddr >> register address
 *  bitStart >> first bit number >> bit numbers 8bit 76543210
 *  bitLength >> number of bytes >> from left to right
 *  dataBits >> bits to be written
 *
 */
uint8 TWI_writebits(uint8 devAddr, uint8 regAddr, uint8 bitStart, uint8 bitLength, uint8 dataBits)
{
	uint8 data = 0;
	status = TWI_recieve_byte_nack(devAddr,regAddr,&data);	// reads register and saves it into data variable

	dataBits <<= (bitStart-bitLength+1); // SHIFT bits to correct positions
	uint8 mask = ((1 << bitLength) - 1) << (bitStart - bitLength + 1);
    data &= mask; // zero all non-important bits in data
    data &= ~(mask); // zero all important bits in existing byte
    data |= data; // combine data with existing byte

    TWI_send_byte(devAddr,regAddr,data);	// send the data to the register ^_^

    return 0;
}


uint8 TWI_read_bytes(uint8 devAddr,uint8 regAddr,uint8 length, uint8* data)
/*
 * reads number of bytes in burst mode
 * length : number of bytes
 * data : array which will save data into
 */
{
	uint8 i = 0;

		uint8 address = (devAddr<<1);		// 7 bit address with WRITE -> r/w = 0;
		TWI_start();
		if (TWI_getStatus() != TW_START)
			return 10;

		TWI_write(0x68);					// sends device address
		if (TWI_getStatus() != TW_MT_SLA_W_ACK)
		        return 11;
		TWI_write(regAddr);					// sends register address
		if (TWI_getStatus() != TW_MT_DATA_ACK)
		        return 12;
		TWI_start();						// send repeated start bit
		if (TWI_getStatus() != TW_REP_START)
		        return 13;
		address = (devAddr<<1) | (1<<0);	// 7bit address with READ -> r/w = 1
		if (TWI_getStatus() != TW_MT_SLA_R_ACK)
		        return 14;

		TWI_write(address);					// send device address again
		for ( i = 0 ; i < length-1 ; i++)
		{
			data[i] = TWI_read_ack();		// read the byte and save in data array.
		}

		data[i] = TWI_read_nack();		// read last byte and send NACK
		if (TWI_getStatus() != TW_MR_DATA_NACK)
		        return 15;
		TWI_stop(); 						// send stop bit

		return 20;

}

void TWI_write_bytes(uint8 devAddr, uint8 regAddr, uint8 length ,sint16* data) // data should be array or address of single 16 bit variable
/*
 *  writes 2 bytes in a single loop
 *  length = 1 >> send 2 bytes
 *  length = 2 >> send 4 bytes ...etc
 */
{
		uint8 ii = 0;
		uint8 address = (devAddr<<1);      // 7 bit address with bit 0 - > r/w = 0 to write.
		TWI_start();					// send start bit
		TWI_write(address);				// send device address
		TWI_write(regAddr);				// send register address
		for (ii = 0 ; ii < length ; ii++)
		{
		TWI_write(data[ii]>>8);				// send the MSB
		TWI_write((uint8)data[ii]);			// send the LSB
		}
		TWI_stop();						// send stop bit

}





