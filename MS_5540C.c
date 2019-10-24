/*
 * MS_5540C.c
 *
 *  Created on: Oct 24, 2019
 *      Author: Abdallh
 */

#include "MS_5540C.h"

// Spi_Config_Type spi_config = {F_CPU_64,FALLING,DOUBLE};
static uint16 C1,C2,C3,C4,C5,C6; // Paramaters to get from calibration words.

void reset_sensor()
{
	// reset sequence for MS_5540C
	spi_send_byte(0x15);
	spi_send_byte(0x55);
	spi_send_byte(0x40);	// reset it done.

}

void get_calb_parameters( void ) //calculates parameters and calibrations for sensor.
{
	uint16 word[4]=0;		// 4 variables for 4 calibration words.
	spi_send_byte(0x1D);				// get word 1
	spi_send_byte(0x50);
	word[0] = spi_recieve_byte();		// get Higher byte of word
	word[0] = word[0] << 8;				// shift left to put it in HIGH 8 bits
	word[0] =( spi_recieve_byte() ) | ( word[0] );				// get lower byte of word
	reset_sensor();

	spi_send_byte(0x1D);				// get word 2
	spi_send_byte(0x60);
	word[1] = spi_recieve_byte();		// get Higher byte of word
	word[1] = word[1] << 8;				// shift left to put it in HIGH 8 bits
	word[1] =( spi_recieve_byte() ) | ( word[1] );					// get lower byte of word
	reset_sensor();


	spi_send_byte(0x1D);				// get word 3
	spi_send_byte(0x90);
	word[2] = spi_recieve_byte();		// get Higher byte of word
	word[2] = word[2] << 8;				// shift left to put it in HIGH 8 bits
	word[2] =( spi_recieve_byte() ) | ( word[2] );					// get lower byte of word
	reset_sensor();

	spi_send_byte(0x1D);				// get word 4
	spi_send_byte(0x60);
	word[3] = spi_recieve_byte();		// get Higher byte of word
	word[3] = word[3] << 8;				// shift left to put it in HIGH 8 bits
	word[3] = 	word[3] =( spi_recieve_byte() ) | ( word[3] );		// get lower byte of word
	reset_sensor();


	// to get paramters
	 C1 = (word[0]>>1 & 0x7FFF);											// Get C1
	 C2 = ( ( (word[2] & 0x003F )<<6 ) | (word[3] & 0x003F) );			// Get C2
	 C3 = ( (word[3] & 0xFFC0)>>6 );								// Get C3
	 C4 = ( (word[2] & 0xFFC0)>>6 );								// Get C4
	 C5 = ( ( word[1] & 0xFFC0)>>6 ) | ( (word[0] & 0x0001 ) << 10);	// get C5
	 C6 = (word[1] & 0x003F );

}

static uint16 read_Pressure_raw()
{
	reset_sensor();
	uint16 D1 = 0;		// Pressure RAW
	spi_send_byte(0x0F); // send command of getting pressure ( MSB of address )
	spi_send_byte(0x40); // send LSB of address
	_delay_ms(35);		// wait for operation conversion complete
	D1 = ( ( spi_recieve_byte() ) << 8 ); // Receive HIGH Byte of Pressure value
	D1 = ( spi_recieve_byte() | D1 );   // Receive LOW Byte of D1 value and put it into LOW Of D1
	return D1;

}

static uint16 read_temp_raw( void )
{
	reset_sensor();
	uint16 D2 = 0;		// Temperature RAW
	spi_send_byte(0x0F); // send command of getting temperature ( MSB of address )
	spi_send_byte(0x20); // send LSB of address
	_delay_ms(35);		// wait for operation conversion complete
	D2 = ( ( spi_recieve_byte() ) << 8 ); // Receive HIGH Byte of temperature value
	D2 = ( spi_recieve_byte() | D2 );   // Receive LOW Byte of D2 value and put it into LOW Of D2
	return D2;
}

void read_temp( void ) 		// calculation of real values
{
	uint16 D1 = read_Pressure_raw();
	uint16 D2 = read_temp_raw();

	const sint32 UT1 = (C5 << 3) + 20224;
	const sint32 dT = D2 - UT1;
	const sint32 TEMP = 200 + ((dT * (C6 + 50)) >> 10);
	const sint32 OFF  = (C2 * 4) + (((C4 - 512) * dT) >> 12);
	const sint32 SENS = C1 + ((C3 * dT) >> 10) + 24576;
	const sint32 X = (SENS * (D1 - 7168) >> 14) - OFF;
	long PCOMP = ((X * 10) >> 5) + 2500;
	g_Temp_Real = TEMP/10;
	g_PCOMPHG = PCOMP * 750.06 / 10000; // mbar*10 -> mmHg === ((mbar/10)/1000)*750/06

	// Code Upove needs some modifications in order to work in LOW and HIGH temperates ( T<20 & T > 45 )
	long T2,P2;

	if (TEMP < 200) 		// calculate for low temp < 20
	    {
	      T2 = (11 * (C6 + 24) * (200 - TEMP) * (200 - TEMP) ) >> 20;
	      P2 = (3 * T2 * (PCOMP - 3500) ) >> 14;
	    }
	  else if (TEMP > 450)		// calculate for high temp > 45
	    {
	      T2 = (3 * (C6 + 24) * (450 - TEMP) * (450 - TEMP) ) >> 20;
	      P2 = (T2 * (PCOMP - 10000) ) >> 13;
	    }

	  if ( (TEMP < 200) || (TEMP > 450) )		// get final temp
	  {
	    g_Temp_Real = ( TEMP - T2 ) / 10 ;				// Degree
	    g_PCOMPHG = (PCOMP - P2) * 750.06 / 10000;		// mbar*10 -> mmHg === ((mbar/10)/1000)*750/06
	  }
}



