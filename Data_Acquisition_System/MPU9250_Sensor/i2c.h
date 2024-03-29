/*
 * i2c.h
 *
 *  Created on: Oct 23, 2019
 *      Author: Al-Shimaa
 */

#ifndef I2C_H_
#define I2C_H_


#define TW_START         0x08 // start has been sent
#define TW_REP_START     0x10 // repeated start
#define TW_MT_SLA_W_ACK  0x18 // Master transmit ( slave address + Write request ) to slave + Ack received from slave
#define TW_MT_SLA_R_ACK  0x40 // Master transmit ( slave address + Read request ) to slave + Ack received from slave
#define TW_MT_DATA_ACK   0x28 // Master transmit data and ACK has been received from Slave.
#define TW_MR_DATA_ACK   0x50 // Master received data and send ACK to slave
#define TW_MR_DATA_NACK  0x58 // Master received data but doesn't send ACK to slave


#include "micro_config.h"
#include "std_types.h"
#include "common_macros.h"

void TWI_init(void);

void TWI_start(void);

void TWI_stop(void);

void TWI_write(uint8 data);

uint8 TWI_readWithACK(void); //read with send Ack

uint8 TWI_readWithNACK(void); //read without send Ack

uint8 TWI_getStatus(void);


#endif /* I2C_H_ */
