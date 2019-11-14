/*
 * 	TinyRTC Module Source File
 *	Target : ATMEGA32
 *  Created on: Nov 14, 2019
 *  Author: Abdallh
 */


#include "TinyRTC.h"

#ifdef send_serial

char buffer2[12]= {0}; // buffer 2 for itoa function
void print_serial(uint16);

#endif

void RTC_set_time()
/*
 *  This Function Sets time and initialzes the RTC
 *  Sets time to the Defined constants in the header file.
 */
{
	uint8 address = (RTC_ADDR<<1 | I2C_WRITE);    	  //RTC 7 bit address with bit 0 - > r/w = 0 to write.
	TWI_start();									    // send start bit
	TWI_write(address);									// send RTC Address.
	TWI_write(RTC_SEC_ADDR);							// reset the register pointer to seconds register
	TWI_write(dec_to_bcd(SECONDS));						// set seconds register.
	TWI_write(dec_to_bcd(MINUTES));						// set minutes register.
	TWI_write(dec_to_bcd(HOURS));						// set hours register.
	TWI_write(dec_to_bcd(DAY));							// set day register.
	TWI_write(dec_to_bcd(DATE));						// set date register.
	TWI_write(dec_to_bcd(MONTH));						// set month register.
	TWI_write(dec_to_bcd(YEAR));						// set year register.
	TWI_stop();											// send stop bit and release the bus.


}

void RTC_read_time()
/*
 * Reads Current time and date.
 *
 */
{
	uint8 address = (RTC_ADDR<<1 | I2C_WRITE);				// RTC 7 bit address with bit 0 -> r/w = 1 to read.
	TWI_start(); 											// send start bit
	TWI_write(address);										// send device address
	TWI_write(RTC_SEC_ADDR);								// reset the register pointer to seconds register
	TWI_start();											// send repeated start bit
	address = (RTC_ADDR<<1 | I2C_READ);
	TWI_write(address);

	uint8 seconds = bcd_to_dec( TWI_read_ack() & 0x7F  );	// read seconds register
	uint8 minutes = bcd_to_dec( TWI_read_ack()  	   );	// read minutes register
	uint8 hours   = bcd_to_dec( TWI_read_ack() & 0x3F  );	// read hours register
	uint8 day	  = bcd_to_dec( TWI_read_ack()   	   );	// read day register
	uint8 date    = bcd_to_dec( TWI_read_ack()         );	// read date register
	uint8 month   = bcd_to_dec( TWI_read_ack()         );	// read month register
	uint8 year    = bcd_to_dec( TWI_read_nack()         );	// read year register
	TWI_stop();

#ifdef send_serial
	UART_sendByte('\n');
	UART_sendString("Time Now   : ");

	print_serial(hours);
	UART_sendString(" : ");

	print_serial(minutes);
	UART_sendString(" : ");

	print_serial(seconds);

	UART_sendByte('\n');
//	print_serial(day);
	UART_sendString("Date Today : 20");

	print_serial(year);
	UART_sendString(" : ");
	print_serial(month);
	UART_sendString(" : ");
	print_serial(date);

	UART_sendByte('\n');
	UART_sendString("Done_Reading!");
#endif

}

uint8 dec_to_bcd(uint8 value)
/*
 * Converts Decimal to BCD
 * RTC registers are in BCD form.
 */
{

	return ( (value/10*16) + (value%10) );
}

uint8 bcd_to_dec(uint8 value)
/*
 * Converts BCD to Decimal
 */
{
	 return ( (value/16*10) + (value%16) );
}

#ifdef send_serial

void print_serial(uint16 word)
{
	itoa(word,buffer2,10);
	UART_sendString(buffer2);
}

#endif
