/*
 *  rtc_example
 *	Target : ATMEGA32
 *  Created on: Nov 14, 2019
 *  Author: Abdallh
 */


#include "TinyRTC.h"

int main(void)

{
	UART_init();
	TWI_init();
	RTC_set_time();


	while(1)
	{
		RTC_read_time();
		_delay_ms(2000);

	}

}
