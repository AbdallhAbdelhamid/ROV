/*
 * spi.h
 *	Module : Configurable SPI Driver.h
 *  Created on: Oct 23, 2019
 *  Author: Abdallh
 */

#ifndef SPI2_H_
#define SPI2_H_

#include "std_types.h"
#include "common_macros.h"
#include "micro_config.h"

typedef enum
{
	F_CPU_4, F_CPU_16, F_CPU_64, F_CPU_128
}Spi_Clock;

typedef enum
{
	RISING , FALLING
}Spi_Edge_type;

typedef enum
{
	NORMAL,DOUBLE
}Spi_Double_Speed;

typedef struct
{
	Spi_Clock clock;
	Spi_Edge_type edge;
	Spi_Double_Speed speed;

}Spi_Config_Type;

void spi_init_master(Spi_Config_Type* config_ptr);
void spi_init_slave();
void spi_send_byte(const uint8 byte);
uint8 spi_recieve_byte();
void spi_send_string(uint8* string);
void spi_recieve_string(uint8* string);


#endif /* SPI2_H_ */
