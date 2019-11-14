/*
 * 	MS_5540C.c
 *	Module : MS 5540C Sensor Source File
 *	Target : ATMEGA32
 *  Created on: Oct 24, 2019
 *  Author: Abdallh
 */

#include "MS_5540C.h"

// Spi_Config_Type spi_config = {F_CPU_64,FALLING,DOUBLE};
 sint32 g_Temp_Real; // Global variables contains the latest Temperature value
 sint32 g_PCOMPHG; // Global variables contains the latest Pressure value

#ifdef calibrate

uint16 C1,C2,C3,C4,C5,C6; // Paramaters toget from calibration words.

#endif

#ifdef serial_debug

char buffer3[12] = {0};		// only needed for debugging purposes.

void print_serial_L(uint16 word)
/**
 * converts Long integers to strings and sends them via Serial interface
 * input:
 */
{
	UART_sendByte('\n');
	ltoa(word,buffer3,10);	// long integer to string
	UART_sendString(buffer3);
}


void print_serial_S(uint16 word)
/**
 * converts short integers to strings and sends them via Serial interface
 * input:
 */
{
	UART_sendByte('\n');
	itoa(word,buffer3,10);	// short integer to string
	UART_sendString(buffer3);
}


#endif

void reset_sensor()
{
	// reset sequence for MS_5540C
	spi_send_byte(0x15);
	spi_send_byte(0x55);
	spi_send_byte(0x40);	// reset it done.

}

#ifdef CALIBRATE

void ms5540c_get_calb_parameters( void ) //calculates parameters and calibrations for sensor.
{
		UART_sendByte('\n');
		UART_sendByte('S');

	uint8 dummy=0;

	uint16 word[4]={0,0,0,0};		// 4 variables for 4 calibration words.
	spi_send_byte(0x00);				// get word 1
	spi_send_byte(0x1D);				// get word 1
	spi_send_byte(0x50);

	word[0] = spi_recieve_byte(0x00);		// get Higher byte of word
	dummy = spi_recieve_byte(0x00);
	word[0] = word[0] << 8;				// shift left to put it in HIGH 8 bits
	word[0] =( dummy |  word[0] );				// get lower byte of word


	print_serial2_S(word[0]);


	reset_sensor();



	spi_send_byte(0x1D);				// get word 2
	spi_send_byte(0x60);
	/*Another method to get words*/
//	word[1] = spi_recieve_byte(0x00);		// get Higher byte of word
//	word[1] = word[1] << 8;				// shift left to put it in HIGH 8 bits
//	word[1] =( spi_recieve_byte(0x00) ) | ( word[1] );		// get lower byte of word

	word[1] = spi_recieve_byte(0x00);		// get Higher byte of word
	dummy = spi_recieve_byte(0x00);
	word[1] = word[1] << 8;				// shift left to put it in HIGH 8 bits
	word[1] =( dummy |  word[1] );				// get lower byte of word

	reset_sensor();

	print_serial_S(word[1]);

	spi_send_byte(0x1D);				// get word 3
	spi_send_byte(0x90);
	word[2] = spi_recieve_byte(0x00);		// get Higher byte of word
	word[2] = word[2] << 8;				// shift left to put it in HIGH 8 bits
	word[2] =( spi_recieve_byte(0x00) ) | ( word[2] );					// get lower byte of word
	reset_sensor();

	print_serial_S(word[2]);


	spi_send_byte(0x1D);				// get word 4
	spi_send_byte(0xA0);
	word[3] = spi_recieve_byte(0x00);		// get Higher byte of word
	word[3] = word[3] << 8;				// shift left to put it in HIGH 8 bits
	word[3] = 	word[3] =( spi_recieve_byte(0x00) ) | ( word[3] );		// get lower byte of word
	reset_sensor();

	print_serial_S(word[3]);


	// to get paramters
	 C1 = (word[0]>>1 & 0x7FFF);											// Get C1
	 C2 = ( ( (word[2] & 0x003F )<<6 ) | (word[3] & 0x003F) );			// Get C2
	 C3 = ((word[3] >> 6) & 0x03FF );								// Get C3
	 C4 = ((word[2] >> 6) & 0x03FF );								// Get C4
	 C5 = (((word[1	]>> 6) & 0x03FF)) | ( (word[0] & 0x0001 ) << 10);	// get C5
	 C6 = (word[1] & 0x003F );

	 print_serial_S(C1);
	 print_serial_S(C2);
	 print_serial_S(C3);
	 print_serial_S(C4);
	 print_serial_S(C5);
	 print_serial_S(C6);

}

#endif
static uint16 read_Pressure_raw()
{
	reset_sensor();
	uint16 D1 = 0;		// Pressure RAW
	spi_send_byte(0x0F); // send command of getting pressure ( MSB of address )
	spi_send_byte(0x40); // send LSB of address
	_delay_ms(35);		// wait for operation conversion complete
	D1 = ( ( spi_recieve_byte(0x00) ) << 8 ); // Receive HIGH Byte of Pressure value
	D1 = ( spi_recieve_byte(0x00) | D1 );   // Receive LOW Byte of D1 value and put it into LOW Of D1
#ifdef serial_debug
		UART_send_byte('\n');
		UART_sendString("PressureRaw");
		print_serial2(D1);
#endif

	return D1;


}

static uint16 read_temp_raw( void )
{
	reset_sensor();
	uint16 D2 = 0;		// Temperature RAW
	spi_send_byte(0x0F); // send command of getting temperature ( MSB of address )
	spi_send_byte(0x20); // send LSB of address
	_delay_ms(35);		// wait for operation conversion complete
	D2 = ( ( spi_recieve_byte(0x00) ) << 8 ); // Receive HIGH Byte of temperature value
	D2 = ( spi_recieve_byte(0x00) | D2 );   // Receive LOW Byte of D2 value and put it into LOW Of D2
#ifdef serial_debug
	UART_send_byte('\n');
	UART_sendString("TempRaw\n");
	print_serial2(D2);
#endif
	return D2;
}


void MS5540c_read_pressure_temp( void ) 		// calculation of real values
{
	uint16 D1 = read_Pressure_raw();
	uint16 D2 = read_temp_raw();

	const long UT1 = (C5 << 3) + 20224;			// these calculations and equations are from MS5540c datasheet.
	const long dT = D2 - UT1;
	const long TEMP = 200 + ((dT * (C6 + 50)) >> 10);
	const long OFF  = (C2 * 4) + (((C4 - 512) * dT) >> 12);
	const long SENS = C1 + ((C3 * dT) >> 10) + 24576;
	const long X = (SENS * (D1 - 7168) >> 14) - OFF;
	const long PCOMP = ((X * 10) >> 5) + 2500;
	g_Temp_Real = TEMP/10;
	g_PCOMPHG = PCOMP * 750.00 / 10000; // mbar*10 -> mmHg === ((mbar/10)/1000)*750.06
#ifdef serial_debug
	UART_send_byte('\n');
	UART_sendString("PressureComb");
	print_serial2(g_PCOMPHG);
	UART_send_byte('\n');
		UART_sendString("TempComb");
		print_serial2(g_Temp_Real);
#endif
/*	// Code Upove needs some modifications in order to work in LOW and HIGH temperates ( T<20 & T > 45 )
	long T2 = 0 ,P2 = 0;

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

*/
}



