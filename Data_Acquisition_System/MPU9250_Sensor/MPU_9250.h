/*
 *  MPU_9250_Registers.h
 *  Module : MPU 9250 Register header file
 *  Created on: Oct 25, 2019
 *  Author: Abdallh
 */

#ifndef MPU_9250_H_
#define MPU_9250_H_

/********************************* INCLUDES ***********************************/

#include "i2c.h"

/********************************* CALIBRATION *******************************/
/* If you are not  going to Calibrate the MPU9250 ,comment out the next line */

#define CALIBRATE



/****************************** GLOBAL VARIABLES ***********************************/

/* Raw Sensor Readings*/
extern sint16 ax , ay , az;		//
extern sint16 gx , gy , gz; 	//
extern sint16 mx , my , mz;		//

/* sensor adjusted readings */
/* Final Readings */
extern  float32 ax_adjusted , ay_adjusted , az_adjusted;
extern  float32 gx_adjusted , gy_adjusted , gz_adjusted;
extern  float32 mx_adjusted , my_adjusted , mz_adjusted;

/*  Sensor Offset data */
extern sint16 x_gyro_offset , y_gyro_offset , z_gyro_offset;   //
extern sint16 x_accel_offset ,y_accel_offset ,z_accel_offset ; //
extern sint16 x_magnet_sens,y_magnet_sens,z_magnet_sens ;      //

/********************************* Registers *********************************/


#define MPU9250_ADDRESS_AD0_LOW     0x68 // address pin low (GND), default for InvenSense evaluation board
#define MPU9250_ADDRESS_AD0_HIGH    0x69 // address pin high (VCC)
#define MPU9250_DEFAULT_ADDRESS     0x68

#define MPU9250_RA_XG_OFFS_TC       0x00 //[7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU9250_RA_YG_OFFS_TC       0x01 //[7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU9250_RA_ZG_OFFS_TC       0x02 //[7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU9250_RA_X_FINE_GAIN      0x03 //[7:0] X_FINE_GAIN
#define MPU9250_RA_Y_FINE_GAIN      0x04 //[7:0] Y_FINE_GAIN
#define MPU9250_RA_Z_FINE_GAIN      0x05 //[7:0] Z_FINE_GAIN
#define MPU9250_RA_XA_OFFS_H        0x06 //[15:0] XA_OFFS
#define MPU9250_RA_XA_OFFS_L_TC     0x07
#define MPU9250_RA_YA_OFFS_H        0x08 //[15:0] YA_OFFS
#define MPU9250_RA_YA_OFFS_L_TC     0x09
#define MPU9250_RA_ZA_OFFS_H        0x0A //[15:0] ZA_OFFS
#define MPU9250_RA_ZA_OFFS_L_TC     0x0B
#define MPU9250_RA_SELF_TEST_X      0x0D //[7:5] XA_TEST[4-2], [4:0] XG_TEST[4-0]
#define MPU9250_RA_SELF_TEST_Y      0x0E //[7:5] YA_TEST[4-2], [4:0] YG_TEST[4-0]
#define MPU9250_RA_SELF_TEST_Z      0x0F //[7:5] ZA_TEST[4-2], [4:0] ZG_TEST[4-0]
#define MPU9250_RA_SELF_TEST_A      0x10 //[5:4] XA_TEST[1-0], [3:2] YA_TEST[1-0], [1:0] ZA_TEST[1-0]
#define MPU9250_RA_XG_OFFS_USRH     0x13 //[15:0] XG_OFFS_USR
#define MPU9250_RA_XG_OFFS_USRL     0x14
#define MPU9250_RA_YG_OFFS_USRH     0x15 //[15:0] YG_OFFS_USR
#define MPU9250_RA_YG_OFFS_USRL     0x16
#define MPU9250_RA_ZG_OFFS_USRH     0x17 //[15:0] ZG_OFFS_USR
#define MPU9250_RA_ZG_OFFS_USRL     0x18
#define MPU9250_RA_SMPLRT_DIV       0x19
#define MPU9250_RA_CONFIG           0x1A
#define MPU9250_RA_GYRO_CONFIG      0x1B
#define MPU9250_RA_ACCEL_CONFIG     0x1C
#define MPU9250_RA_FF_THR           0x1D
#define MPU9250_RA_FF_DUR           0x1E
#define MPU9250_RA_MOT_THR          0x1F
#define MPU9250_RA_MOT_DUR          0x20
#define MPU9250_RA_ZRMOT_THR        0x21
#define MPU9250_RA_ZRMOT_DUR        0x22
#define MPU9250_RA_FIFO_EN          0x23
#define MPU9250_RA_I2C_MST_CTRL     0x24
#define MPU9250_RA_I2C_SLV0_ADDR    0x25
#define MPU9250_RA_I2C_SLV0_REG     0x26
#define MPU9250_RA_I2C_SLV0_CTRL    0x27
#define MPU9250_RA_I2C_SLV1_ADDR    0x28
#define MPU9250_RA_I2C_SLV1_REG     0x29
#define MPU9250_RA_I2C_SLV1_CTRL    0x2A
#define MPU9250_RA_I2C_SLV2_ADDR    0x2B
#define MPU9250_RA_I2C_SLV2_REG     0x2C
#define MPU9250_RA_I2C_SLV2_CTRL    0x2D
#define MPU9250_RA_I2C_SLV3_ADDR    0x2E
#define MPU9250_RA_I2C_SLV3_REG     0x2F
#define MPU9250_RA_I2C_SLV3_CTRL    0x30
#define MPU9250_RA_I2C_SLV4_ADDR    0x31
#define MPU9250_RA_I2C_SLV4_REG     0x32
#define MPU9250_RA_I2C_SLV4_DO      0x33
#define MPU9250_RA_I2C_SLV4_CTRL    0x34
#define MPU9250_RA_I2C_SLV4_DI      0x35
#define MPU9250_RA_I2C_MST_STATUS   0x36
#define MPU9250_RA_INT_PIN_CFG      0x37
#define MPU9250_RA_INT_ENABLE       0x38
#define MPU9250_RA_DMP_INT_STATUS   0x39
#define MPU9250_RA_INT_STATUS       0x3A
#define MPU9250_RA_ACCEL_XOUT_H     0x3B
#define MPU9250_RA_ACCEL_XOUT_L     0x3C
#define MPU9250_RA_ACCEL_YOUT_H     0x3D
#define MPU9250_RA_ACCEL_YOUT_L     0x3E
#define MPU9250_RA_ACCEL_ZOUT_H     0x3F
#define MPU9250_RA_ACCEL_ZOUT_L     0x40
#define MPU9250_RA_TEMP_OUT_H       0x41
#define MPU9250_RA_TEMP_OUT_L       0x42
#define MPU9250_RA_GYRO_XOUT_H      0x43
#define MPU9250_RA_GYRO_XOUT_L      0x44
#define MPU9250_RA_GYRO_YOUT_H      0x45
#define MPU9250_RA_GYRO_YOUT_L      0x46
#define MPU9250_RA_GYRO_ZOUT_H      0x47
#define MPU9250_RA_GYRO_ZOUT_L      0x48
#define MPU9250_RA_EXT_SENS_DATA_00 0x49
#define MPU9250_RA_EXT_SENS_DATA_01 0x4A
#define MPU9250_RA_EXT_SENS_DATA_02 0x4B
#define MPU9250_RA_EXT_SENS_DATA_03 0x4C
#define MPU9250_RA_EXT_SENS_DATA_04 0x4D
#define MPU9250_RA_EXT_SENS_DATA_05 0x4E
#define MPU9250_RA_EXT_SENS_DATA_06 0x4F
#define MPU9250_RA_EXT_SENS_DATA_07 0x50
#define MPU9250_RA_EXT_SENS_DATA_08 0x51
#define MPU9250_RA_EXT_SENS_DATA_09 0x52
#define MPU9250_RA_EXT_SENS_DATA_10 0x53
#define MPU9250_RA_EXT_SENS_DATA_11 0x54
#define MPU9250_RA_EXT_SENS_DATA_12 0x55
#define MPU9250_RA_EXT_SENS_DATA_13 0x56
#define MPU9250_RA_EXT_SENS_DATA_14 0x57
#define MPU9250_RA_EXT_SENS_DATA_15 0x58
#define MPU9250_RA_EXT_SENS_DATA_16 0x59
#define MPU9250_RA_EXT_SENS_DATA_17 0x5A
#define MPU9250_RA_EXT_SENS_DATA_18 0x5B
#define MPU9250_RA_EXT_SENS_DATA_19 0x5C
#define MPU9250_RA_EXT_SENS_DATA_20 0x5D
#define MPU9250_RA_EXT_SENS_DATA_21 0x5E
#define MPU9250_RA_EXT_SENS_DATA_22 0x5F
#define MPU9250_RA_EXT_SENS_DATA_23 0x60
#define MPU9250_RA_MOT_DETECT_STATUS    0x61
#define MPU9250_RA_I2C_SLV0_DO      0x63
#define MPU9250_RA_I2C_SLV1_DO      0x64
#define MPU9250_RA_I2C_SLV2_DO      0x65
#define MPU9250_RA_I2C_SLV3_DO      0x66
#define MPU9250_RA_I2C_MST_DELAY_CTRL   0x67
#define MPU9250_RA_SIGNAL_PATH_RESET    0x68
#define MPU9250_RA_MOT_DETECT_CTRL      0x69
#define MPU9250_RA_USER_CTRL        0x6A
#define MPU9250_RA_PWR_MGMT_1       0x6B
#define MPU9250_RA_PWR_MGMT_2       0x6C
#define MPU9250_RA_BANK_SEL         0x6D
#define MPU9250_RA_MEM_START_ADDR   0x6E
#define MPU9250_RA_MEM_R_W          0x6F
#define MPU9250_RA_DMP_CFG_1        0x70
#define MPU9250_RA_DMP_CFG_2        0x71
#define MPU9250_RA_FIFO_COUNTH      0x72
#define MPU9250_RA_FIFO_COUNTL      0x73
#define MPU9250_RA_FIFO_R_W         0x74
#define MPU9250_RA_WHO_AM_I         0x75

// power and clock setting

#define MPU9250_PWR1_SLEEP_BIT          6
#define MPU9250_PWR1_CLKSEL_BIT 		2

#define MPU9250_PWR1_SLEEP_BIT_LENGTH   1
#define MPU9250_PWR1_CLKSEL_LENGTH		3

#define MPU9250_CLOCK_INTERNAL          0
#define MPU9250_CLOCK_PLL         		1

#define MPU9250_PWR1_DEVICE_RESET_BIT   7
#define MPU9250_PWR1_DEVICE_RESET_LENGTH   1

#define MPU9250_USERCTRL_SIG_COND_RESET_BIT 0
#define MPU9250_USERCTRL_SIG_COND_RESET_LENGTH 1

// read through .. (bypass I2C master mode )
#define MPU9250_USERCTRL_I2C_ENABLE_BIT			5
#define MPU9250_USERCTRL_I2C_ENABLE_BIT_LENGTH  1

#define MPU9250_RA_INT_PIN_CFG_BIT				1
#define MPU9250_RA_INT_PIN_CFG_BIT_LENGTH		1



// Gyro parameters

#define MPU9250_GYRO_SCALE_FACTOR_FS_250		131
#define MPU9250_GYRO_SCALE_FACTOR_FS_500		65.5
#define MPU9250_GYRO_SCALE_FACTOR_FS_1000		32.8
#define MPU9250_GYRO_SCALE_FACTOR_FS_2000		16.4


#define MPU9250_GCONFIG_FS_SEL_BIT 	    4
#define MPU9250_GCONFIG_FS_SEL_LENGTH   2

// Gyro Sensitivity

#define MPU9250_GYRO_FS_250				0x00
#define MPU9250_GYRO_FS_500				0x01
#define MPU9250_GYRO_FS_1000			0x02
#define MPU9250_GYRO_FS_2000			0x03


// Accel parameters

#define MPU9250_ACCEL_SCALE_FACTOR_AFS_2		16384
#define MPU9250_ACCEL_SCALE_FACTOR_AFS_4		8192
#define MPU9250_ACCEL_SCALE_FACTOR_AFS_8		4096
#define MPU9250_ACCEL_SCALE_FACTOR_AFS_16		2048

#define MPU9250_ACONFIG_XA_ST_BIT           7
#define MPU9250_ACONFIG_YA_ST_BIT           6
#define MPU9250_ACONFIG_ZA_ST_BIT           5
#define MPU9250_ACONFIG_AFS_SEL_BIT         4
#define MPU9250_ACONFIG_AFS_SEL_LENGTH      2
#define MPU9250_ACONFIG_ACCEL_HPF_BIT       2
#define MPU9250_ACONFIG_ACCEL_HPF_LENGTH    3

// Accel Sensisivity

#define MPU9250_ACCEL_FS_2          0x00
#define MPU9250_ACCEL_FS_4          0x01
#define MPU9250_ACCEL_FS_8          0x02
#define MPU9250_ACCEL_FS_16         0x03

// general config

#define MPU9250_CFG_DLPF_CFG_BIT 			2
#define MPU9250_CFG_DLPF_CFG_LENGTH			3

// I2C Master clock

#define MPU9250_I2C_MST_CLK_BIT     		3
#define MPU9250_I2C_MST_CLK_LENGTH 		    4

// accel offsets

#define MPU9250_X_ACCEL_OFFSET				0x77
#define MPU9250_Y_ACCEL_OFFSET				0x7A
#define MPU9250_Z_ACCEL_OFFSET				0X7D

// magnetometer

#define MPU9250_AK8963_ADDRESS 					0x48
#define MPU9250_AK8963_XR_A_H					0x03
#define MPU9250_AK8963_YR_A_H					0x05
#define MPU9250_AK8963_ZR_A_H					0x07
#define MPU9250_AK8963_CTRL_R_A					0x0A
#define MPU9250_AK8963_CTRL_MODE_BIT			 4
#define MPU9250_AK8963_CTRL_MODE_LENGTH			 4
#define MPU9250_AK8963_X_SENS_R					0x10
#define MPU9250_AK8963_Y_SENS_R					0x11
#define MPU9250_AK8963_Z_SENS_R					0x12
#define MPU9250_AK8963_CTRL_BITS_NUMBER			4
#define MPU9250_AK8963_CTRL_BITS_NUMBER_LENGTH 	1

#define MPU9250_AK8963_MAX_RANGE 				32760

// magnet power modes

#define FUSE_MODE			15
#define CONTINUOUS_MODE	 	2
#define CONTINUOUS_MODE_2	6
#define POWER_DOWN_MODE     0

// magnet offsets and scales
// hard offests = 0 and soft offset = 1 if no calibrations were made
#define MAGNET_HARD_OFFSET_X					0
#define MAGNET_HARD_OFFSET_Y					0
#define MAGNET_HARD_OFFSET_Z					0

#define MAGNET_SOFT_SCALE_X						1
#define MAGNET_SOFT_SCALE_Y						1
#define MAGNET_SOFT_SCALE_Z						1


/************************************* Funcions ****************************************/

void MPU9250_init();					// initializing the MPU6500 //
void MPU9250_AK8963_init();				// initialized Magnetometer
void MPU9250_set_clock();
void set_full_scale_gyro_range(uint8 range); // sets gyro full scale
void set_full_scale_accel_range(uint8 range);// sets accel full scale
void setSleepEnabled (uint8 enabled); 		 // sets sleep mode for sensor
void set_rate(uint8 rate);					 // sets gyro sampling rate
void set_PDLF_mode(uint8 mode);				//  sets digital low pass filter configuration and determines the internal sampling rate used by the device.
void reset(); 								// resets the device

void get_motion6();							// get raw readings from Accel + Gyro
void get_magnet();							// get raw readings from magnet
void get_motion9();							// get raw reading from Accel + Gyro + magnet

void get_motion6_adjusted();				// gets real values from Accel + Gyro
void get_magnet_adjusted();					// gets real values from magnet
void get_motion9_adjusted();				// gets real values from Accel + Gyro + magnet

void set_magnet_mode(uint8 mode);			// sets mode for magnet
void get_sensitivity_magnet();				// reads sensitivity offsets from magneto registers

void calibrate_gyro_accel();				// calibrates gyro and accel and saves offests into variable and corresponding registers
void calibrate_magnet();					// gets soft & hard interferance saved into the corresponding global regiters.

void set_accel_offsets(uint16* xa_offset, uint16* ya_offset , uint16* za_offset  );// sets X Y Z Accel offsets
void set_xaccel_offset(uint16* offset);	// set X Accel Offset
void set_yaccel_offset(uint16* offset); // set Y Accel Offset
void set_zaccel_offset(uint16* offset);	// set Z Accel Offset

void set_gyro_offsets(uint16* xg_offset, uint16* yg_offset , uint16* zg_offset  ); // sets X Y Z Gyro offsets
void set_xgyro_offset(uint16* offset); // set X gyro offset
void set_ygyro_offset(uint16* offset); // set y gyro offset
void set_zgyro_offset(uint16* offset); // set z gyro offset

void get_gyro_offsets();  // gets x,y,z gyro offsets
void get_gyro_x_offset(); // reads Gyro x offset
void get_gyro_y_offset(); // reads Gyro y offset
void get_gyro_z_offset(); // reads Gyro z offset

void get_accel_offsets();  // gets x y z accel offsets
void get_accel_x_offset(); // reads accel x offset
void get_accel_x_offset(); // reads accel y offset
void get_accel_x_offset(); // reads accel z offset





#endif /* MPU_9250_H_ */
