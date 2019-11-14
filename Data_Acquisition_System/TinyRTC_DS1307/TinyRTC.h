/*
 * TinyRTC.h
 *
 *  Created on: Nov 14, 2019
 *      Author: Al-Shimaa
 */

#ifndef TINYRTC_H_
#define TINYRTC_H_

// if u dont want to send output over serial interface , comment out next line

#define send_serial

/********************************* INCLUDES ***********************************/
#include "common_macros.h"
#include "micro_config.h"
#include "std_types.h"
#include <stdlib.h>
#include "i2c.h"

#ifdef send_serial
#include "uart.h"
#endif

/********************************** Defines *****************************************/
#define RTC_ADDR 			0x68
#define RTC_SEC_ADDR		0x00
#define RTC_MIN_ADDR		0x01
#define RTC_HOUR_ADDR		0x02
#define RTC_DAY_ADDR		0x03
#define RTC_DATE_ADDR		0x04
#define RTC_MONTH_ADDR		0x05
#define RTC_YEAR_ADDR		0x06
#define RTC_CONTROL_ADDR	0x07

/******************************* Time Setup Defines **********************************/

#define SECONDS			00
#define MINUTES			59
#define HOURS			20
#define DAY				6
#define DATE			14
#define MONTH			11
#define YEAR			19

/***************************** Function Prototypes ***************************************/

void RTC_set_time(void);							// Sets  current time and date.
void RTC_read_time(void);							// Reads Current time and date.
uint8 bcd_to_dec(uint8 value);						// converts from bcd to decimal
uint8 dec_to_bcd(uint8 value);						// converts from decimal to bcd



#endif /* TINYRTC_H_ */
