/*
 * MS_5540C.h
 *
 *  Created on: Oct 24, 2019
 *      Author: Abdallh
 */

#ifndef MS_5540C_H_
#define MS_5540C_H_

#include "common_macros.h"
#include "micro_config.h"
#include "std_types.h"
#include "spi2.h"

extern float32 g_Temp_Real; // Global variables contains the latest Temperature value
extern float32 g_PCOMPHG; // Global variables contains the latest Pressure value


void get_calb_parameters( void ); // calculates parameters and calibrations for sensor. initialize SPI before.
void read_temp( void ); // Calculate TEMP and Pressure and puts values in global variables g_Temp_Real, g_PCOMPHG


#endif /* MS_5540C_H_ */
