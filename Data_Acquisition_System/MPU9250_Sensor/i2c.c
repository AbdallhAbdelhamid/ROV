/*
 * i2c.c
 *
 *  Created on: Oct 23, 2019
 *      Author: Al-Shimaa
 */


#include "i2c.h"

void TWI_init()
{
	TWBR = 0x03;
	TWSR |= (1<<TWPS0);
	/*
	 *  Prescaler = 4 and TWBR = 3 with FCPU = 16MHZ >>> bit rate = 400,000 kps.
	 *
	 */
	TWCR = (1<<TWEN);
	/*
	 *  TWEN = 1 >> enable TWI module
	 *  TWie= 0 >> disable interrupts.
	 */
	TWAR |= (1<<1);
	/* Sets this MC as 0x0000001
	 * TWGCE = 0 >> General call is off
	 *
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
	TWCR = (1<<TWINT) | (1<<TWSTO) | (1<<TWEN);
	/*
		 * TWINT = 1  >> Clear the flag before sending start bit.
		 * TWISTA = 1 >> Send the stop bit
		 * TWEN = 1 >> Enables module
		 */
	while(BIT_IS_CLEAR(TWCR,TWINT)) {} // wait until stop bit is send already.


}

void TWI_write(uint8 data)
{
	TWDR = data; // put data into the register
	TWCR = (1 << TWINT) | (1 << TWEN); // clear the flag and enable the module.
	while (BIT_IS_CLEAR(TWCR,TWINT)) {}// wait until data sent successfully.

}

uint8 TWI_read_with_ack(void)
{
	TWCR = (1<<TWINT) | (1<< TWEN) | (1<<TWEA); // clear flag and enable module and enable sending ACK
	while(BIT_IS_CLEAR(TWCR,TWINT)) {}
	return TWDR;
}

uint8 TWI_read_with_nack(void)
{
	TWCR = (1<<TWINT) | (1<< TWEN); // clear flag and enable module
	while(BIT_IS_CLEAR(TWCR,TWINT)) {}
	return TWDR;
}

uint8 TWI_get_status(void)
{
	uint8 status = TWSR & 0xF8;
	return (status);
}
