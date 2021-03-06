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
#include "src/event_scheduler.h"

extern double ch0_val, ch1_val;
extern uint8_t command_flag;

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

	I2CSPM_Init(&i2cInit);	/*Does not include any blocking inside it*/
	I2C_IntEnable( I2C0, I2C_IEN_RXDATAV | I2C_IEN_TXC | I2C_IEN_RXFULL );
	NVIC_EnableIRQ( I2C0_IRQn );
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

	data_tx[0] = command;
	data_tx[1] = data;
	/* Setup I2C Transfer Sequence for transmit data */
	{
		seq_tx.addr	=	device_address << 1;
		seq_tx.flags	=	flag;
		seq_tx.buf[0].data = &data_tx[0];
		seq_tx.buf[1].data = &data_tx[1];
		seq_tx.buf[0].len	=	1;
		seq_tx.buf[1].len	=	1;
	};

	I2C_TransferReturn_TypeDef return_type = I2C_TransferInit( I2C0, &seq_tx );
	if (return_type != i2cTransferInProgress)
	{
		LOG_ERROR("Failed");
	}
	else
		LOG_INFO("Transferring");
}



/*
 * Write a single byte on the I2C bus.
 * Accomplished using Flag - I2C_FLAG_WRITE
 */
void I2C_send_byte(uint8_t device_address, uint8_t reg_addr, uint8_t flag)
{

	uint8_t data_tx[1] ={0};
	data_tx[0] = reg_addr;
	LOG_INFO("SENDING BYTE");
	/* Setup I2C Transfer Sequence for transmit data */
	{
		seq_tx.addr	=	device_address << 1;
		seq_tx.flags	=	flag;
		seq_tx.buf[0].data	=	&data_tx[0];
		seq_tx.buf[0].len	=	1;
	};

	I2C_TransferInit( I2C0, &seq_tx );

}

/* Read word (2 bytes) from the I2C bus
 * Note - For lux sensor, Lower byte is read first and then the Higher Byte, as opposed to the
 * inbuilt temperature/humidity sensor on the Blue Gecko Board
 */
void I2C_read_word(uint8_t device_address, uint8_t command)
{
	data_rx = command;

	{
		/* Setup I2C Transfer Sequence for receiving data */
		seq_rx.addr	=	device_address << 1;
		seq_rx.flags	=	I2C_FLAG_WRITE_READ;
		seq_rx.buf[0].data	=	&data_rx;
		seq_rx.buf[0].len	=	1;
		seq_rx.buf[1].data	=	data_rx_1;
		seq_rx.buf[1].len	=	2;
	}
	I2C_TransferInit( I2C0, &seq_rx );
}



/*
 * Store the lower data byte and upper data byte returned after Read Word operation
 * into 1 16-bit buffer
 */
double read_lux_register(void)
{
	/* 	Storing received data in a 16-bit variable */
	data_buffer = 0;
	data_buffer	=	data_rx_1[1];
	data_buffer	= data_buffer << 8;
	data_buffer	|=	data_rx_1[0];

	double final_data = (double)(data_buffer);
	LOG_INFO("Raw_Value = %lf", final_data);
	return final_data;
}


/*
 * Calculates final lux data based on above mentioned functions and I2C APIs
 * Refer Datasheet of APDS-9301 for more detailed explanation of calculation.
 * values from ADC Channel 0 and ADC Channel 1 are converted into one lux value
 */
double get_lux_sensor_values(double ch0_val, double ch1_val)
{
	double luxVal = 0.0;
	LOG_INFO("FINAL CH0 = %lf", ch0_val);
	LOG_INFO("FINAL CH1 = %lf", ch1_val);
	if ((ch1_val != 0) && (ch0_val != 0))	//Make sure no "divide by zero" Exception occurs!
	{
		double ratio = ch1_val / ch0_val;
		/* Refer Lux Sensor Datasheet for detailed explanation of below calculation*/

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
	}
	return luxVal;

}


/* @I2C0 interrupt handler
 * This IRQ handler is called when I2C transfer is in process or completed.
 * When for the first time, I2C_send_command() is called to power on, command_flag remains
 * 0 and does not call the acquire_lux_data() state machine. Then, command_flag is set to 1
 * and events are generated for the I2C state machine
 *
 * @param	none
 * @return	none
 */
void I2C0_IRQHandler(void)
{
	LOG_INFO("in I2C IRQ");
	I2C_TransferReturn_TypeDef I2C_transfer_return_status = I2C_Transfer( I2C0 );
	CORE_DECLARE_IRQ_STATE;
	if(I2C_transfer_return_status == i2cTransferDone)
	{
		//set event of i2c transfer complete
		CORE_ENTER_CRITICAL();
		if(command_flag == 1)
		{
			event_name.EVENT_I2C_TRANSFER_COMPLETE = true;
			event_name.EVENT_NONE = false;
			gecko_external_signal(event_name.EVENT_I2C_TRANSFER_COMPLETE);
		}
		command_flag = 1;

		CORE_EXIT_CRITICAL();
		LOG_INFO("Transfer success");
		LOG_INFO("IN I2C ISR");
	}
	else if(I2C_transfer_return_status != i2cTransferInProgress){
		//set event of i2c transfer error
		CORE_ENTER_CRITICAL();
		event_name.EVENT_I2C_TRANSFER_ERROR = 1;
		event_name.EVENT_NONE = false;
		CORE_EXIT_CRITICAL();
		LOG_INFO("Transfer FAILED\n");
	}
}
