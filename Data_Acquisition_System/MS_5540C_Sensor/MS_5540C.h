/*
 *
 *	Module : MS 5540C Sensor Header File
 *	Target : ATMEGA32
 *  Created on: Oct 24, 2019
 *  Author: Abdallh
 */

#ifndef MS_5540C_H_
#define MS_5540C_H_


/*If you are not in calibrate mode , Comment out next line  */

//#define CALIBRATE

/*If you are not in serial debugging mode , Comment out next line  */

//#define serial_debug


/************************************INCLUDES ***********************************/

#include "common_macros.h"
#include "micro_config.h"
#include "std_types.h"
#include "spi.h"

#ifdef serial_debug
#include "uart.h"
#include <stdlib.h>
#endif
/******************************** Global Variables************************************/
extern sint32 g_Temp_Real; // Global variables contains the latest Temperature value
extern sint32 g_PCOMPHG; // Global variables contains the latest Pressure value

#ifdef calibrate
extern uint16 C1,C2,C3,C4,C5,C6;
#endif
/************************************ FUNCTIONS *****************************/
void ms5540c_get_calb_parameters( void ); // calculates parameters and calibrations for sensor. initialize SPI before.
void MS5540c_read_pressure_temp( void ); // Calculate TEMP and Pressure and puts values in global variables g_Temp_Real, g_PCOMPHG

/******************************Calibration Words*************************************/
#define WORD1 42845
#define WORD2 14489
#define WORD3 32091
#define WORD4 44804

#define C1 21422
#define C2 1732
#define C3 700
#define C4 501
#define C5 1250
#define C6 25


#endif /* MS_5540C_H_ */
