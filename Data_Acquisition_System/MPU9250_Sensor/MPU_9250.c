/*
 *  MPU_9250.c
 *  Module : 9250 Sensor.c
 *  Created on: Oct 26, 2019
 *  Author: Abdallh
 */

#include "MPU_9250.h"
/*  Sensor output data  */

 sint16 ax , ay , az;		//
 sint16 gx , gy , gz; 		//
 sint16 mx , my , mz;		// global variables to  get sensor reading


 /* sensor adjusted readings */

  float32 ax_adjusted , ay_adjusted , az_adjusted;
  float32 gx_adjusted , gy_adjusted , gz_adjusted;
  float32 mx_adjusted , my_adjusted , mz_adjusted;		//  reading after adjusting >> final output

 uint16 buffer[20] = 0;

 /* transformation matrices to transform the accel and gyro axes to match the magnetometer axes*/
 uint8 trans_x[3] = {0, 1, 0};
 uint8 trans_y[3] = {1, 0, 0};
 uint8 trans_z[3] = {0, 0,-1};


/*  Sensor Offset data */
 sint16 x_gyro_offset , y_gyro_offset , z_gyro_offset;   //
 sint16 x_accel_offset ,y_accel_offset ,z_accel_offset ; //
 sint16 x_magnet_sens,y_magnet_sens,z_magnet_sens ;      // to store offsets


#ifdef CALIBRATE

 /* calibration data */

 /* gyro and accel */
 uint16 buffersize = 1000;		// amount of reading used to average
 uint8 acel_deadzone = 8;		// Acelerometer error allowed
 uint8 giro_deadzone = 1;		// Gyro error allowed

 sint16 mean_ax,mean_ay,mean_az,mean_gx,mean_gy,mean_gz,state=0 ;

 /* magnetometer */

 sint16  max_mag_x, min_mag_x, max_mag_y, min_mag_y, max_mag_z, min_mag_z;

 // u need to read and save the following 6 variables in a #define numbers because it will be cleared after sysetem reset
 float32 magnet_hard_offset_x = 0 ,magnet_hard_offset_y = 0 ,magnet_hard_offset_z = 0 ;  // hard iron interferance offset
 float32 magnet_scale_x = 0 , magnet_scale_y = 0 , magnet_scale_z = 0 ;					 // soft iron interferance scale

#endif

void MPU9250_init()						// initializing the MPU6500 //
{
	set_clock();						// sets clock to gyro internal oscillator
	set_full_scale_gyro_range(MPU9250_GYRO_FS_250); // sets full scale range of gyro to 250DPS
	set_full_scale_accel_range(MPU9250_ACCEL_FS_2); // sets full scale range of accel to 2g
	set_sleep_mode(FALSE);							// disables sleep mode
	MPU9250_AK8963_init();
}

void MPU9250_AK8963_init()
/*
 * initializes Magnetometer
 * Continuous mode 2 : 100HZ , 16 bit ADC
 * enables bypass to read the sensor directly from i2c bus
 */
{
	set_magnet_mode(CONTINUOUS_MODE_2);				// enables magnet
	enable_bypass(TRUE);							// enables bypass mode
}

void MPU9250_set_clock()
/*
 *  sets power options for MPU9250 CLOCK
 * 	Sets power to Option 1 : auto select clock source , PLL if ready , else , use internal
 * 	this uses gyro oscillator  as it's more stable that the internal one.
 */
{
	TWI_writebits(MPU9250_DEFAULT_ADDRESS,MPU9250_RA_PWR_MGMT_1, MPU9250_PWR1_CLKSEL_BIT  ,MPU9250_PWR1_CLKSEL_LENGTH, MPU9250_CLOCK_PLL);
}


void set_full_scale_gyro_range(uint8 range)
/*
 *  Sets the full scale range for Gyro : available inputs are :
 *  MPU9250_GYRO_FS_250 , MPU9250_GYRO_FS_500 , MPU9250_GYRO_FS_1000, MPU9250_GYRO_FS_2000
 *
 */
{
	TWI_writebits(MPU9250_DEFAULT_ADDRESS,MPU9250_RA_GYRO_CONFIG,MPU9250_GCONFIG_FS_SEL_BIT,MPU9250_GCONFIG_FS_SEL_LENGTH,range);

}

void set_full_scale_accel_range(uint8 range)
/*
 * Sets the full scale range for Gyro : available inputs are :
 *  MPU9250_ACCEL_FS_2 , MPU9250_ACCEL_FS_4 , MPU9250_ACCEL_FS_8, MPU9250_ACCEL_FS_16
 */
{
	TWI_writebits(MPU9250_DEFAULT_ADDRESS,MPU9250_RA_ACCEL_CONFIG,MPU9250_ACONFIG_AFS_SEL_BIT,MPU9250_ACONFIG_AFS_SEL_LENGTH,range);
}

void setSleepEnabled (uint8 enabled)
	// sets sleep mode for device
{
	TWI_writebits(MPU9250_DEFAULT_ADDRESS,MPU9250_RA_PWR_MGMT_1,MPU9250_PWR1_SLEEP_BIT,MPU9250_PWR1_SLEEP_BIT_LENGTH,enabled);

}


void set_rate(uint8 rate)
/*	sets the sample rate for Gyro
 *  note : Accel output rate is 1KHZ & gyro output rate is either 8KHZ or 1KHZ
 *  all operations are done duo do sample rate
 *  so if gyro output rate >>>> gyro rate then duplicate of gyro outputs into the registers will happen
 *  Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
 */
{
	TWI_send_byte(MPU9250_DEFAULT_ADDRESS,MPU9250_RA_SMPLRT_DIV,rate);
}

void set_PDLF_mode(uint8 mode)
/*
 *  sets digital low pass filter configuration and determines the internal sampling
 *  rate used by the device.
 *  Hint : Max ACCEL out rate = 4kHz
 *
 *          |   ACCELEROMETER    |           GYROSCOPE
 * DLPF_CFG | Bandwidth | Delay  | Bandwidth | Delay  | Sample Rate
 * ---------+-----------+--------+-----------+--------+-------------
 * 0        | 260Hz     | 0ms    | 256Hz     | 0.98ms | 8kHz
 * 1        | 184Hz     | 2.0ms  | 188Hz     | 1.9ms  | 1kHz
 * 2        | 94Hz      | 3.0ms  | 98Hz      | 2.8ms  | 1kHz
 * 3        | 44Hz      | 4.9ms  | 42Hz      | 4.8ms  | 1kHz
 * 4        | 21Hz      | 8.5ms  | 20Hz      | 8.3ms  | 1kHz
 * 5        | 10Hz      | 13.8ms | 10Hz      | 13.4ms | 1kHz
 * 6        | 5Hz       | 19.0ms | 5Hz       | 18.6ms | 1kHz
 * 7        |   -- Reserved --   |   -- Reserved --   | Reserved
 */
{
	TWI_writebits(MPU9250_DEFAULT_ADDRESS, MPU9250_RA_CONFIG, MPU9250_CFG_DLPF_CFG_BIT , MPU9250_CFG_DLPF_CFG_LENGTH, mode);
}

void set_master_clock_speed(uint8 speed)
/* Set I2C master clock speed.
 * * I2C_MST_CLK | I2C Master Clock Speed | 8MHz Clock Divider
 * ------------+------------------------+-------------------
 * 0           | 348kHz                 | 23
 * 1           | 333kHz                 | 24
 * 2           | 320kHz                 | 25
 * 3           | 308kHz                 | 26
 * 4           | 296kHz                 | 27
 * 5           | 286kHz                 | 28
 * 6           | 276kHz                 | 29
 * 7           | 267kHz                 | 30
 * 8           | 258kHz                 | 31
 * 9           | 500kHz                 | 16
 * 10          | 471kHz                 | 17
 * 11          | 444kHz                 | 18
 * 12          | 421kHz                 | 19
 * 13          | 400kHz                 | 20
 * 14          | 381kHz                 | 21
 * 15          | 364kHz                 | 22
 *
 *
 */
{
	TWI_writebits(MPU9250_DEFAULT_ADDRESS, MPU9250_RA_I2C_MST_CTRL, MPU9250_I2C_MST_CLK_BIT , MPU9250_I2C_MST_CLK_LENGTH , speed);
}

void reset_sensors()
/*
 * Resets all sensors and signals and sensor registers
 */
{
	TWI_writebits(MPU9250_DEFAULT_ADDRESS,MPU9250_RA_USER_CTRL,MPU9250_USERCTRL_SIG_COND_RESET_BIT,MPU9250_USERCTRL_SIG_COND_RESET_LENGTH,TRUE);

}

void reset()
/*
 * Resets all device
 */
{
	TWI_writebits(MPU9250_DEFAULT_ADDRESS,MPU9250_RA_PWR_MGMT_1,MPU9250_PWR1_DEVICE_RESET_BIT,MPU9250_PWR1_DEVICE_RESET_LENGTH,TRUE);

}

void get_device_id() // gets device id , should be 0x34
{
	// not implemented yet
}

void get_motion6()
/*
 * get motion for gyro and accel sensors and store in ax ay az gx gy gz
 *
 */
{
	TWI_read_bytes(MPU9250_DEFAULT_ADDRESS, MPU9250_RA_ACCEL_XOUT_H, 14 , buffer ); // reads number of bytes from external device registers

		*ax = (((uint16)buffer[0]) << 8) | buffer[1];
	    *ay = (((uint16)buffer[2]) << 8) | buffer[3];
	    *az = (((uint16)buffer[4]) << 8) | buffer[5];
	    *gx = (((uint16)buffer[8]) << 8) | buffer[9];
	    *gy = (((uint16)buffer[10]) << 8) | buffer[11];
	    *gz = (((uint16)buffer[12]) << 8) | buffer[13];
}

void get_motion9()
{
	get_motion6();
	get_magnet();
}


void get_motion6_adjusted()
/*
 * Gets final readings of sesnors , divides each reading with corresponding scale factor to convert to normal units.
 * Gyro  unit : degree/s
 * Accel uint : g
 * Hint : Scale factors MUST be changed with each sensitivity set of gyro/accel.
 * 		 this is used with Gyro and Accel most sensitivte modes. (FS = 250 & AFS = 2 )
 */
{
	float32 gx_adjusted_temp,gy_adjusted_temp,gz_adjusted_temp,ax_adjusted_temp,ay_adjusted_temp,az_adjusted_temp;
	gx_adjusted_temp = (float)gx / (float)MPU9250_GYRO_SCALE_FACTOR_FS_250 ; // x unit : degree/s
	gy_adjusted_temp = (float)gy / (float)MPU9250_GYRO_SCALE_FACTOR_FS_250 ; // y uint : degree/s
	gz_adjusted_temp = (float)gz / (float)MPU9250_GYRO_SCALE_FACTOR_FS_250 ; // z uint : degree/s
	ax_adjusted_temp = (float)ax / (float)MPU9250_ACCEL_SCALE_FACTOR_AFS_2 ; // x unit : g
	ay_adjusted_temp = (float)ay / (float)MPU9250_ACCEL_SCALE_FACTOR_AFS_2 ; // y unit : g
	az_adjusted_temp = (float)az / (float)MPU9250_ACCEL_SCALE_FACTOR_AFS_2 ; // z unit : g

	//  transform the accel and gyro axes to match the magnetometer axes
	gx_adjusted = gx_adjusted_temp *trans_x[0] + gy_adjusted_temp * trans_x[1] + gz_adjusted_temp* trans_x[2] ;
	gy_adjusted = gx_adjusted_temp *trans_y[0] + gy_adjusted_temp * trans_y[1] + gz_adjusted_temp* trans_y[2] ;
	gz_adjusted = gx_adjusted_temp *trans_z[0] + gy_adjusted_temp * trans_z[1] + gz_adjusted_temp* trans_z[2] ;

	ax_adjusted = ax_adjusted_temp * trans_x[0]+ ay_adjusted_temp * trans_x[1] + az_adjusted_temp* trans_x[2];
	ay_adjusted = ax_adjusted_temp * trans_y[0]+ ay_adjusted_temp * trans_y[1] + az_adjusted_temp* trans_y[2];
	az_adjusted = ax_adjusted_temp * trans_z[0]+ ay_adjusted_temp * trans_z[1] + az_adjusted_temp* trans_z[2];




}

void get_motion9_adjusted()
{
	get_motion6_adjusted();
	get_magnet_adjusted();
}




void set_xaccel_offset(uint16* offset)	// set X Offset
{
	TWI_write_bytes(MPU9250_DEFAULT_ADDRESS,MPU9250_X_ACCEL_OFFSET, 1 , &offset);
}

void set_yaccel_offset(uint16* offset)// set Y offset
{
	TWI_write_bytes(MPU9250_DEFAULT_ADDRESS,MPU9250_Y_ACCEL_OFFSET, 1 , &offset);
}

void set_zaccel_offset(uint16* offset)// set Z offset
{
	TWI_write_bytes(MPU9250_DEFAULT_ADDRESS,MPU9250_Z_ACCEL_OFFSET, 1 , &offset);
}

void set_accel_offsets(uint16* xa_offset, uint16* ya_offset , uint16* za_offset  )
/*
 * sets offsets for X , Y , Z Accel.
 */
{
	set_xaccel_offset(&xa_offset);
	set_yaccel_offset(&ya_offset);
	set_zaccel_offset(&za_offset);

}

void set_xgyro_offset(uint16* offset) // set X gyro offset
{
	TWI_write_bytes(MPU9250_DEFAULT_ADDRESS,MPU9250_RA_XG_OFFS_USRH, 1 , &offset);

}

void set_ygyro_offset(uint16* offset)// set Y gyro offset
{
	TWI_write_bytes(MPU9250_DEFAULT_ADDRESS,MPU9250_RA_YG_OFFS_USRH, 1 , &offset);

}

void set_zgyro_offset(uint16* offset)// set Z gyro offset
{
	TWI_write_bytes(MPU9250_DEFAULT_ADDRESS,MPU9250_RA_ZG_OFFS_USRH, 1 , &offset);

}


void set_gyro_offsets(uint16* xg_offset, uint16* yg_offset , uint16* zg_offset  )
/*
 * sets offsets for X , Y , Z gyro.
 */
{
	set_xgyro_offset(&xg_offset);
	set_ygyro_offset(&yg_offset);
	set_zgyro_offset(&zg_offset);

}

void get_accel_x_offset() // reads x offset
{
	TWI_read_bytes(MPU9250_DEFAULT_ADDRESS,MPU9250_X_ACCEL_OFFSET,2,buffer);
	x_accel_offset = ( (uint16)buffer[0]<<8 )|(buffer[1] );
}

void get_accel_y_offset() // reads y offset
{
	TWI_read_bytes(MPU9250_DEFAULT_ADDRESS,MPU9250_Y_ACCEL_OFFSET,2,buffer);
	y_accel_offset = ( (uint16)buffer[0]<<8 )|(buffer[1] );
}

void get_accel_z_offset() // reads z offset
{
	TWI_read_bytes(MPU9250_DEFAULT_ADDRESS,MPU9250_Z_ACCEL_OFFSET,2,buffer);
	z_accel_offset = ( (uint16)buffer[0]<<8 )|(buffer[1] );
}

void get_gyro_x_offset() // reads x offset
{
	TWI_read_bytes(MPU9250_DEFAULT_ADDRESS,MPU9250_RA_XG_OFFS_USRH,2,buffer);
	x_gyro_offset = ( (uint16)buffer[0]<<8 )|(buffer[1] );
}

void get_gyro_y_offset() // reads y offset
{
	TWI_read_bytes(MPU9250_DEFAULT_ADDRESS,MPU9250_RA_YG_OFFS_USRH,2,buffer);
	y_gyro_offset = ( (uint16)buffer[0]<<8 )|(buffer[1] );
}

void get_gyro_z_offset() // reads z offset
{
	TWI_read_bytes(MPU9250_DEFAULT_ADDRESS,MPU9250_RA_ZG_OFFS_USRH,2,buffer);
	z_gyro_offset = ( (uint16)buffer[0]<<8 )|(buffer[1] );
}

void get_gyro_offsets()		// gets x,y,z gyro offsets
{
	get_gyro_x_offset();
	get_gyro_y_offset();
	get_gyro_z_offset();

}

void get_accel_offsets()	// get x y z accel offsets
{
	get_accel_x_offset();
	get_accel_y_offset();
	get_accel_z_offset();

}

/***************************************** MAGNET ****************************************/

void get_magnet()			// get x y z magnetic reading , read 7 bytes instead of 6 because ST2 register MUST be read

{
	TWI_read_bytes(MPU9250_AK8963_ADDRESS, MPU9250_AK8963_XR_A_H, 7 , buffer );
	*mx =  buffer[0] | (((uint16)buffer[1]) << 8);
	*my =  buffer[2] | (((uint16)buffer[3]) << 8);
	*mz =  buffer[4] | (((uint16)buffer[5]) << 8);

}


void set_magnet_mode(uint8 mode)
/*
 *  Set mode for magnet
 *  Inputs:
 *  		0 > No power
 *  		1 > Single measurement
 *			2 > Continuous measurement 1
 *  		6 > Continuous measurement 2
 *  		4 > External trigger measurement node
 *  		8 > Self Test Mode
 *  		15> Fuse ROM mode | FUSE_MODE
 */
{

	TWI_writebits(MPU9250_AK8963_ADDRESS, MPU9250_AK8963_CTRL_R_A,  MPU9250_AK8963_CTRL_MODE_BIT,  MPU9250_AK8963_CTRL_MODE_LENGTH, mode );
	_delay_ms(100); // delay 100 for each mode change to take effect
	TWI_writebits(MPU9250_AK8963_ADDRESS, MPU9250_AK8963_CTRL_R_A,  MPU9250_AK8963_CTRL_BITS_NUMBER,  MPU9250_AK8963_CTRL_BITS_NUMBER_LENGTH, TRUE ); // set output to 16 bit

}

void get_sensitivity_magnet()
/*
 * gets sensitivity adjustment values and saves them
 * in the respected global variables
 * should be done at least once
 */
{
	set_magnet_mode(POWER_DOWN_MODE); // we have to move into power down mode before changing to any other mode
	_delay_ms(100);					  // long wait until mode changes
	set_magnet_mode(FUSE_MODE);		// turn into FUSE MODE to be able to read Sensitivity bits
	_delay_ms(100);
	TWI_recieve_byte_nack(MPU9250_AK8963_ADDRESS,MPU9250_AK8963_X_SENS_R,&x_magnet_sens);	// read X sensitivity
	TWI_recieve_byte_nack(MPU9250_AK8963_ADDRESS,MPU9250_AK8963_Y_SENS_R,&y_magnet_sens);	// read y sensitivity
	TWI_recieve_byte_nack(MPU9250_AK8963_ADDRESS,MPU9250_AK8963_Z_SENS_R,&z_magnet_sens);	// read z sensitivity
	set_magnet_mode(CONTINUOUS_MODE); //  go back to normal mode.
	_delay_ms(100);
}


void get_magnet_adjusted(void)
/*
 * adjusted magnet reading and convert to magnetix flux density
 * Unit : Nano tesla
 */
{
	mx_adjusted =(float)( ((mx-MAGNET_HARD_OFFSET_X ) * MAGNET_SOFT_SCALE_X) * ((( (x_magnet_sens - 128) * 0.5 ) / 128 ) +1 ))  * (float)(0.1499)   ; // 0.1499 is sentivity factor for magnet
	my_adjusted =(float)( ((my-MAGNET_HARD_OFFSET_Y ) * MAGNET_SOFT_SCALE_Y) * ((( (y_magnet_sens - 128) * 0.5 ) / 128 ) +1 ))  * (float)(0.1499)	;// sensitivity = 4912/32760 = 0.1499
	mz_adjusted =(float)( ((mz-MAGNET_HARD_OFFSET_Z ) * MAGNET_SOFT_SCALE_Z) * ((( (z_magnet_sens - 128) * 0.5 ) / 128 ) +1 ))  * (float)(0.1499) 	;
}

static void enable_bypass(uint8 state)
/*
 * enables bypass mode
 * when enabled : can talk directly to magnetometer from MASTER microcontroller
 * inputs : TRUE : enable bypass mode
 * 			FALSE: disable bypass mode
 */
{

	if(state==TRUE)
		{
			TWI_writebits(MPU9250_DEFAULT_ADDRESS,MPU9250_RA_USER_CTRL,MPU9250_USERCTRL_I2C_ENABLE_BIT,MPU9250_USERCTRL_I2C_ENABLE_BIT_LENGTH,FALSE); // disables i2c master mode.. needed to be off for bypass mode.
			TWI_writebits(MPU9250_DEFAULT_ADDRESS,MPU9250_RA_INT_PIN_CFG,MPU9250_RA_INT_PIN_CFG_BIT,MPU9250_RA_INT_PIN_CFG_BIT_LENGTH,TRUE);		  // enables bypass mode
		}
	else
		{
			TWI_writebits(MPU9250_DEFAULT_ADDRESS,MPU9250_RA_USER_CTRL,MPU9250_USERCTRL_I2C_ENABLE_BIT,MPU9250_USERCTRL_I2C_ENABLE_BIT_LENGTH,FALSE); // enables i2c master mode..
			TWI_writebits(MPU9250_DEFAULT_ADDRESS,MPU9250_RA_INT_PIN_CFG,MPU9250_RA_INT_PIN_CFG_BIT,MPU9250_RA_INT_PIN_CFG_BIT_LENGTH,TRUE);		  // disables bypass mode
		}

}

/************************************** Calibrations  ************************************/
#ifdef CALIBRATE

void calibrate_gyro_accel()
/*
 * Calibration functions for Gyro and Accel sensors
 * calculates gyro and accel offsets
 * offsets are saved into global offset registers
 * offsets also are saved into corresponding registers
 * to eliminate them from future readings.
 * should be used only at least ONCE.
 */
{



	uint8 offset_zero = 0;

	/* set offsets to zero */

	set_gyro_offsets(&offset_zero,&offset_zero,&offset_zero);
	set_accel_offsets(&offset_zero,&offset_zero,&offset_zero);

	if(state == 0 )
	{
		mean_sensors_gyro_accel();
		state++;
	}
	if(state == 1)
	{
		calibrate_sensors();
	}
	set_accel_offsets(&x_accel_offset,&y_accel_offset,&z_accel_offset);	// add offsets into the registers
	set_gyro_offsets(&x_gyro_offset,&y_gyro_offset,&z_gyro_offset);		// add offsets into the registers


}


static void mean_sensors_gyro_accel(void)  // gets mean reading from sensors
{
	 sint32 i=0,buff_ax=0,buff_ay=0,buff_az=0,buff_gx=0,buff_gy=0,buff_gz=0;
	 sint16 x =0;

	 while(i<(buffersize+100) )
	 {
		 get_motion6(); // get motion from 6 degrees
	 }

	 if (i>100 && i<=(buffersize+100)) //First 100 measures are discarded
	 {
	      buff_ax=buff_ax+ax;
	      buff_ay=buff_ay+ay;
	      buff_az=buff_az+az;
	      buff_gx=buff_gx+gx;
	      buff_gy=buff_gy+gy;
	      buff_gz=buff_gz+gz;
	 }

	 if (i==(buffersize+100))
	 {
	      mean_ax=buff_ax/buffersize;
	      mean_ay=buff_ay/buffersize;
	      mean_az=buff_az/buffersize;
	      mean_gx=buff_gx/buffersize;
	      mean_gy=buff_gy/buffersize;
	      mean_gz=buff_gz/buffersize;
	     }
	     i++;			// increase counter to take next reading
	     _delay_ms(2); // delay 2 milles so we don't get repeated readings


}


static void calibrate_sensors()
{
	x_accel_offset=-mean_ax/8;
	y_accel_offset=-mean_ay/8;
	z_accel_offset=(16384-mean_az)/8;

	x_gyro_offset=-mean_gx/4;
    y_gyro_offset=-mean_gy/4;
    z_gyro_offset=-mean_gz/4;
    uint8 ready = 0;
    while(1)
    {
    		ready=0;

    	    set_x_accel_offset(&x_accel_offset);
    	    set_y_accel_offset(&y_accel_offset);
    	    set_z_accel_offset(&z_accel_offset);

    	    set_xgyro_offset(&x_gyro_offset);
    	    set_ygyro_offset(&y_gyro_offset);
    	    set_zgyro_offset(&z_gyro_offset);

    	    mean_sensors_gyro_accel();

    	    if (abs(mean_ax)<=acel_deadzone) ready++;
    	     else x_accel_offset=x_accel_offset-mean_ax/acel_deadzone;

    	     if (abs(mean_ay)<=acel_deadzone) ready++;
    	     else y_accel_offset=y_accel_offset-mean_ay/acel_deadzone;

    	     if (abs(16384-mean_az)<=acel_deadzone) ready++;
    	     else z_accel_offset=z_accel_offset+(16384-mean_az)/acel_deadzone;

    	     if (abs(mean_gx)<=giro_deadzone) ready++;
    	     else x_gyro_offset=x_gyro_offset-mean_gx/(giro_deadzone+1);

    	     if (abs(mean_gy)<=giro_deadzone) ready++;
    	     else y_gyro_offset=y_gyro_offset-mean_gy/(giro_deadzone+1);

    	     if (abs(mean_gz)<=giro_deadzone) ready++;
    	     else z_gyro_offset=z_gyro_offset-mean_gz/(giro_deadzone+1);

    	     if (ready==6) break;



    }



}


void calibrate_magnet()
/*
 *  calibration takes around 15 seconds
 *  sensor should be moved in an eight figure ( 8 ) for the 15 seconds to collect samples.
 *
 */
{
	  float32 avg_delta = 0;
	  float32 magnet_avg_delta_x = 0 ,magnet_avg_delta_y = 0 ,magnet_avg_delta_z = 0 ; // for calculation of soft iron interferance

	// set max and min range values
    max_mag_x = max_mag_y = max_mag_z = -MPU9250_AK8963_MAX_RANGE;
    min_mag_x = min_mag_y = min_mag_z = +MPU9250_AK8963_MAX_RANGE;

    measure_range_magnet();

    //compute the hard-iron interference
       magnet_hard_offset_x =  ((float)( max_mag_x + min_mag_x ) / (float)2.0);
       magnet_hard_offset_y =  ((float)( max_mag_y + min_mag_y ) / (float)2.0);
       magnet_hard_offset_z =  ((float)( max_mag_z + min_mag_z ) / (float)2.0);

     //compute the soft-iron interference
       magnet_avg_delta_x = (float)(max_mag_x - min_mag_x)/ (float)2.0 ;
       magnet_avg_delta_y = (float)(max_mag_y - min_mag_y)/ (float)2.0 ;
       magnet_avg_delta_z = (float)(max_mag_z - min_mag_z)/ (float)2.0 ;

       avg_delta = ( magnet_avg_delta_x + magnet_avg_delta_y + magnet_avg_delta_z ) / (float)3;

       magnet_scale_x = (float)avg_delta / (float)magnet_avg_delta_x ;
       magnet_scale_y = (float)avg_delta / (float)magnet_avg_delta_x ;
       magnet_scale_z = (float)avg_delta / (float)magnet_avg_delta_x ;


}

static void measure_range_magnet(){
    uint16 n = 0;
    while(n<1500){

        get_magnet();

        if(mx>max_mag_x)
            max_mag_x = mx;

        if(mx<min_mag_x)
            min_mag_x = mx;

        if(my>max_mag_y)
            max_mag_y = my;

        if(my<min_mag_y)
            min_mag_y = my;

        if(mz>max_mag_z)
            max_mag_z = mz;

        if(mz<min_mag_z)
            min_mag_z = mz;

        n++;
        _delay_ms(20);
    }
}

#endif




