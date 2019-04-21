/*
 * i2c.h
 *
 *  Created on: 20 Apr 2019
 *      Author: TanmayC
 */

#ifndef I2C_H_

#include "em_i2c.h"
#include <i2cspm.h>

/*Function Prototypes*/
void I2C_Data_Transfer(void);
void I2C_init(void);
void I2C_send_command(uint8_t device_address, uint8_t command, uint8_t flag, uint8_t data);
void I2C_send_byte(uint8_t device_address, uint8_t data_tx, uint8_t flag);
double get_lux_byte_data(uint8_t device_address, uint8_t command, uint8_t flag);
double I2C_read_word(uint8_t device_address);
double get_lux_sensor_values(void);


/*Macros*/
#define	LUX_SENSOR_ADDR			( 0x39 )	//for Light sensor
#define	LUX_CONTROL_REG			( 0x00 )
#define	LUX_COMMAND_BIT			( 0x80 )
#define	LUX_POWER_ON			( 0x03 )
#define LUX_DATA0LOW_REG		( 0x0C )
#define LUX_DATA0HIGH_REG		( 0x0D )
#define LUX_DATA1LOW_REG		( 0x0E )
#define LUX_DATA1HIGH_REG		( 0x0F )

//uint8_t	data_tx;
uint8_t	data_rx[2];
uint8_t data_rx_1;
uint16_t data_buffer;
uint16_t temp_Celcius;

I2C_TransferSeq_TypeDef  seq_tx, seq_rx;


#endif /* I2C_H_ */
