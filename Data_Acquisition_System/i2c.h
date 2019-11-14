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
#define ERROR 0

#define I2C_READ		1
#define I2C_WRITE		0



#include "micro_config.h"
#include "std_types.h"
#include "common_macros.h"

extern uint8 status;



void TWI_init(void);

void TWI_start(void);

void TWI_stop(void);

void TWI_write(uint8 data);

uint8 TWI_read_ack(void); //read with send Ack

uint8 TWI_read_nack(void); //read without send Ack

uint8 TWI_getStatus(void);

uint8 TWI_writebits(uint8 devAddr, uint8 regAddr, uint8 bitStart, uint8 bitLength, uint8 dataBits); // writes specfic bits in external device register

uint8 TWI_recieve_byte_nack(uint8 devAddr,uint8 regAddr, uint8* data); // recives a byte from register in external device register

void TWI_send_byte(uint8 devAddr,uint8 regAddr,uint8 byte); // sends byte to external device register

uint8 TWI_read_bytes(uint8 devAddr,uint8 regAddr,uint8 length, uint8 * data); // reads number of bytes from external device registers

void TWI_write_bytes(uint8 devAddr, uint8 regAddr, uint8 length ,sint16* data); // write number of bytes to external device , data should be array or adress of single 16 bit variable


#endif /* I2C_H_ */
