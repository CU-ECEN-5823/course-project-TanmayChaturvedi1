/*
 * i2c.h
 *
 *  Created on: 20 Apr 2019
 *      Author: TanmayC
 */

#ifndef I2C_H_

#include "em_i2c.h"
#include <i2cspm.h>
#include "stdbool.h"
#include "em_core.h"

/*Function Prototypes*/
void I2C_Data_Transfer(void);
void I2C_init(void);
void I2C_send_command(uint8_t device_address, uint8_t command, uint8_t flag, uint8_t data);
void I2C_send_byte(uint8_t device_address, uint8_t reg_addr, uint8_t flag);
void I2C_read_word(uint8_t device_address, uint8_t command);
double get_lux_sensor_values(double, double);
double read_lux_register(void);


/*Macros*/
#define	LUX_SENSOR_ADDR			( 0x39 )	//for Light sensor
#define	LUX_CONTROL_REG			( 0x00 )
#define	LUX_COMMAND_BIT			( 0x80 )
#define	LUX_POWER_ON			( 0x03 )
#define LUX_DATA0LOW_REG		( 0x0C )
#define LUX_DATA0HIGH_REG		( 0x0D )
#define LUX_DATA1LOW_REG		( 0x0E )
#define LUX_DATA1HIGH_REG		( 0x0F )

/*Global Variables*/
uint16_t data_buffer;	//to store final converted lux value acquired from I2C data register
uint8_t data_tx[2];		//used when sending data to I2C-based sensor during I2C_WRITE_WRITE_FLAG operation
uint8_t data_rx;		//Used when receiving word (2Bytes) of data using I2C_READ_FLAG operation
uint8_t data_rx_1[2];	//Used when receiving data after command/data is sent using I2C_WRITE_READ_FLAG. data_rx_1[0]stores Lower data byte for the lux sensor
double ch0, ch1;		//Stores data returned from read_lux_register()

/*Transfer sequence instance for I2C read/write operations*/
I2C_TransferSeq_TypeDef  seq_tx;
I2C_TransferSeq_TypeDef  seq_rx;


#endif /* I2C_H_ */
