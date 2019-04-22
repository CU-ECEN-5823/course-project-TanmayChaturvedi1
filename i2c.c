/*
 * i2c.c
 *
 *  Created on: 20 Apr 2019
 *      Author: TanmayC
 *
 *      For APDS-9301 I2C-based Luminous intensity sensor
 */

#include <i2cspm.h>
#include "i2c.h"
#include "src/log.h"
#include <math.h>


/*
 * Initializes I2C pins for sensor interfacing
 * Need to be called before the while(1) in main()
 */
void I2C_init(void)
{
	I2CSPM_Init_TypeDef i2cInit = I2CSPM_INIT_DEFAULT;
	{
		i2cInit.portLocationScl	=	14;
		i2cInit.portLocationSda	=	16;
		i2cInit.sclPin	=	10;
		i2cInit.sclPort	=	gpioPortC;
		i2cInit.sdaPin	=	11;
		i2cInit.sdaPort	=	gpioPortC;
	};
	I2CSPM_Init(&i2cInit);
}

/*
 * Specific to lux sensor. As per the datasheet, command (0x80 | 0x00) needs to be sent that includes
 * command instruction and a pointer to the command register, along with data (0x3) that indicates
 * a power on condition for the lux sensor
 * Accomplished using flag - I2C_FLAG_WRITE_WRITE
 */
void I2C_send_command(uint8_t device_address, uint8_t command, uint8_t flag, uint8_t data)
{

	LOG_INFO("POWER ON COMMAND SEND");
	uint8_t data_tx[2] ={0};
	data_tx[0] = command;
	data_tx[1] = data;
	/* Setup I2C Transfer Sequence for transmit data */
	//I2C_TransferSeq_TypeDef  seq_tx;
	{
		seq_tx.addr	=	device_address << 1;
		seq_tx.flags	=	flag;
		seq_tx.buf[0].data = &data_tx;
		seq_tx.buf[0].len	=	2;
	};

	I2C_TransferReturn_TypeDef Transfer_Ret_Status = I2CSPM_Transfer( I2C0, &seq_tx );

	if ( Transfer_Ret_Status != i2cTransferDone )
	{
		/* log error, return error code of any unexpected situation */
		LOG_ERROR("Failed in I2C Transfer with return status = %d", Transfer_Ret_Status);
	}

	else if ( Transfer_Ret_Status == i2cTransferDone )
	{
		/* Transfer done, continue */
		LOG_INFO("DATA TRANSFERRED");
	}
}

/*
 * Write a single byte on the I2C bus.
 * Accomplished using Flag - I2C_FLAG_WRITE
 */
void I2C_send_byte(uint8_t device_address, uint8_t data_tx, uint8_t flag)
{

	LOG_INFO("SENDING BYTE");
	/* Setup I2C Transfer Sequence for transmit data */
	//I2C_TransferSeq_TypeDef  seq_tx;
	{
		seq_tx.addr	=	device_address << 1;
		seq_tx.flags	=	flag;
		seq_tx.buf[0].data	=	&data_tx;
		seq_tx.buf[0].len	=	1;
	};

	I2C_TransferReturn_TypeDef Transfer_Ret_Status = I2CSPM_Transfer( I2C0, &seq_tx );

	if ( Transfer_Ret_Status != i2cTransferDone )
	{
		/* log error, return error code of any unexpected situation */
		LOG_ERROR("Failed in I2C Transfer with return status = %d", Transfer_Ret_Status);
	}

	else if ( Transfer_Ret_Status == i2cTransferDone )
	{
		/* Transfer done, continue */
		LOG_INFO("DATA TRANSFERRED");
	}
}

/* Read word (2 bytes) from the I2C bus
 * Note - For lux sensor, Lower byte is read first and then the Higher Byte, as opposed to the
 * inbuilt temperature/humidity sensor on the Blue Gecko Board
 */
double I2C_read_word(uint8_t device_address)
{
	//I2C_TransferSeq_TypeDef  seq_rx;
	{
	/* Setup I2C Transfer Sequence for receiving data */
		seq_rx.addr	=	device_address << 1;
		seq_rx.flags	=	I2C_FLAG_READ;
		seq_rx.buf[0].data	=	data_rx;
		seq_rx.buf[0].len	=	2;
	}

	I2C_TransferReturn_TypeDef Transfer_Ret_Status_RX = I2CSPM_Transfer( I2C0, &seq_rx );

	if ( Transfer_Ret_Status_RX != i2cTransferDone )
	{
		/* log error, return error code of any unexpected situation */
		LOG_ERROR("Failed in I2C Receive Sequence with return status", Transfer_Ret_Status_RX);
	}
	else if ( Transfer_Ret_Status_RX == i2cTransferDone )
	{
		/* I2C Receive done, continue  */
		LOG_INFO("DATA RECEIVED");
	}

	/* 	Storing received data in a 16-bit variable */
	data_buffer	=	data_rx[1];
	data_buffer	= data_buffer << 8;
	data_buffer	|=	data_rx[0];

	double final_data = (double)(data_buffer);
	LOG_INFO("Raw_Value = %lf", final_data);
	return final_data;
}

/*
 * I2C routine to write data and read from the sensor
 */
double get_lux_byte_data(uint8_t device_address, uint8_t command, uint8_t flag)
{
	I2C_send_byte(device_address, command, flag);	//for command and control operation to set register address
	double ret_val = I2C_read_word(device_address);
	return ret_val;
}


/*
 * Calculates final lux data based on above mentioned functions and I2C APIs
 * Refer Datasheet of APDS-9301 for more detailed explanation of calculation
 */
double get_lux_sensor_values(void)
{
	double ch0_val = get_lux_byte_data(LUX_SENSOR_ADDR, LUX_COMMAND_BIT | LUX_DATA0LOW_REG, I2C_FLAG_WRITE) ;
	char name[30];
	sprintf(name, "new lux = %lf", ch0_val);
	LOG_INFO("yo - %s", name);
	double ch1_val = get_lux_byte_data(LUX_SENSOR_ADDR, LUX_COMMAND_BIT | LUX_DATA1LOW_REG, I2C_FLAG_WRITE) ;
	double ratio = ch1_val / ch0_val;

	/* Refer Lux Sensor Datasheet for detailed explanation of below calculation*/
	double luxVal = 0.0;
	if ((ratio <= 0.5) && (ratio > 0.0))
	{
		luxVal = (0.0304 * ch0_val) - ((0.062 * ch0_val) * (pow((ch1_val/ch0_val), 1.4)));
	}
	else if ((ratio <= 0.61) && (ratio > 0.5))
	{
		luxVal = (0.0224 * ch0_val) - (0.031 * ch1_val);
	}
	else if ((ratio <= 0.80) && (ratio > 0.61))
	{
		luxVal = (0.0128 * ch0_val) - (0.0153 * ch1_val);
	}
	else if ((ratio <= 1.3) && (ratio > 0.8))
	{
		luxVal = (0.00146 * ch0_val) - (0.00112*ch1_val);
	}
	else
		luxVal = 0.0;

	return luxVal;
}
