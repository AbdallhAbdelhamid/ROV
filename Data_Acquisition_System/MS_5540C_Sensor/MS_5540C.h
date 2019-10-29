/*
 *
 *	Module : MS 5540C Sensor.h
 *  Created on: Oct 24, 2019
 *      Author: Abdallh
 */

#ifndef MS_5540C_H_
#define MS_5540C_H_


/*If you are not in calibrate mode , Comment out next line  */

#define CALIBRATE

/************************************INCLUDES ***********************************/

#include "common_macros.h"
#include "micro_config.h"
#include "std_types.h"
#include "spi2.h"

/******************************** Global Variables************************************/
extern float32 g_Temp_Real; // Global variables contains the latest Temperature value
extern float32 g_PCOMPHG; // Global variables contains the latest Pressure value

/************************************ FUNCTIONS *****************************/
void get_calb_parameters( void ); // calculates parameters and calibrations for sensor. initialize SPI before.
void read_temp( void ); // Calculate TEMP and Pressure and puts values in global variables g_Temp_Real, g_PCOMPHG


#endif /* MS_5540C_H_ */
