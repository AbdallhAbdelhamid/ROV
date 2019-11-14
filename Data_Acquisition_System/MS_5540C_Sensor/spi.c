/*
 * 	spi.c
 *	Module : Configurable SPI  Driver Source File
 *	Target : ATMEGA32
 *  Created on: Oct 23, 2019
 *  Author: Abdallh
 */

#include "spi.h"



void spi_init_master(Spi_Config_Type* config_ptr)
{

		DDRB = DDRB | (1<<PB4);		// set as output		SS
		DDRB = DDRB | (1<<PB5);		// set as output		MOSI
		DDRB = DDRB & ~(1<<PB6);	// set as input			MISO
		DDRB = DDRB | (1<<PB7);		// set as output		SCK

		SPCR = (1<<SPE) | (1<<MSTR); // Enable SPI Module & set as master.

		/* SPIE = 0 >> disable interrupts
		 * DORD = 0 >> MSB letter is transmitted first.
		 * CPOL = 0
		 */
		SPSR = ( (SPSR & 0xFE ) | (config_ptr-> speed) );   // enable or disables double speed
		SPCR = ( (SPCR & 0xFB ) | ( (config_ptr-> edge)<<CPHA ) ); // Choose sampling on rising or falling edge
		SPCR = ( (SPCR & 0xFC ) | (config_ptr-> clock) ) ; // set the clock



}

void spi_init_slave()
{

		DDRB = DDRB & (~(1<<PB4));  // set as input		SS
		DDRB = DDRB & (~(1<<PB5));	// set as input		MOSI
		DDRB = DDRB | (1<<PB6);		// set as output	MOSI
		DDRB = DDRB & (~(1<<PB7));	// set as input		SCK

	SPCR = (1<<SPE);
	/*
	 * SPE = 1 >> Enable module.
	 * MSTR = 0 >> set as slave.
	 * CLK  = FOSC /4
	 */
}

void spi_send_byte(const uint8 byte)
{
	SPDR = byte;
	while (BIT_IS_CLEAR(SPSR,SPIF)) {} // wait until SPI finish the transmit.
}

uint8 spi_recieve_byte(uint8 byte)
{
	SPDR = byte;					   //
	while (BIT_IS_CLEAR(SPSR,SPIF)) {} // wait until SPI fully reads the data
	return SPDR;
}

void spi_send_string(uint8* string)
{
	uint8 i = 0;
	while (string[i] != '\0')
	{
		spi_send_byte(string[i]);
		i++;
	}
}

void spi_recieve_string(uint8* string)
{
	uint8 i = 0;
	string[i] = spi_recieve_byte(0x00);
	while(string[i]!= '#' )
	{
		i++;
		string[i]=spi_recieve_byte(0x00);
	}
	string[i] = '\0';

}




