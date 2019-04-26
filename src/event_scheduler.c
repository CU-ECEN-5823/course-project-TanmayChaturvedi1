/*
 * event_scheduler.c
 *
 *  Created on: 24 Apr 2019
 *      Author: TanmayC
 */
#include "event_scheduler.h"

extern double lux_value;

// For storing max data in PS
extern uint16_t max_lux_val;

enum lux_sensor_state current_state = LUX_SENSOR_WAIT_FOR_I2C_COMMAND_COMPLETE;

void acquire_lux_data(uint32_t ext_signal)
{
	CORE_DECLARE_IRQ_STATE;

	switch( current_state ){ /*need definition*/



	case LUX_SENSOR_WAIT_FOR_I2C_COMMAND_COMPLETE:
		if ( (event_name.EVENT_INITIATE_STATE_MACHINE ) && ext_signal  /*&& (command_flag == 1)*/ )
		{

			CORE_ENTER_CRITICAL();

			event_name.EVENT_I2C_TRANSFER_COMPLETE = false;
			event_name.EVENT_INITIATE_STATE_MACHINE = false;
			event_name.EVENT_NONE = true;
			CORE_EXIT_CRITICAL();
			if (buffer_flag == 0)
			{
				I2C_read_word(LUX_SENSOR_ADDR, LUX_COMMAND_BIT | LUX_DATA0LOW_REG);	//for command and control operation to set register address
				nonblock_timerWaitUs(420000);
				current_state = LUX_SENSOR_WAIT_FOR_I2C_WRITE_READ_COMPLETE;	//first acquire ch0 val using write + read, then ch1_val using same method
				LOG_INFO("In LUX_SENSOR_WAIT_FOR_I2C_WRITE_READ_COMPLETE STATE, 1st byte");
			}
			else if (buffer_flag == 1)
			{
				I2C_read_word(LUX_SENSOR_ADDR, LUX_COMMAND_BIT | LUX_DATA1LOW_REG);	//for command and control operation to set register address
				nonblock_timerWaitUs(420000);
				current_state = LUX_SENSOR_WAIT_FOR_I2C_WRITE_READ_COMPLETE;	//State remains same to acquire the second byte of data
				LOG_INFO("In LUX_SENSOR_WAIT_FOR_I2C_WRITE_READ_COMPLETE STATE, 2nd byte");
			}

		}
		break;

	case LUX_SENSOR_WAIT_FOR_I2C_WRITE_READ_COMPLETE:
		if ( ( event_name.EVENT_SETUP_TIMER_EXPIRED )   && ext_signal)
		{
			CORE_ENTER_CRITICAL();
			event_name.EVENT_SETUP_TIMER_EXPIRED = false;
			event_name.EVENT_NONE = true;
			CORE_EXIT_CRITICAL();

			if (buffer_flag == 0)
			{
				ch0 = read_lux_register();
				buffer_flag = 1;
				current_state = LUX_SENSOR_WAIT_FOR_I2C_COMMAND_COMPLETE;
				LOG_INFO("In LUX_SENSOR_WAIT_FOR_I2C_WRITE_READ_COMPLETE STATE for ch0 = %lf", ch0);
				event_name.EVENT_INITIATE_STATE_MACHINE = true;
				event_name.EVENT_NONE = false;
				acquire_lux_data(START_LUX_STATE_MACHINE);
			}
			else if (buffer_flag == 1)
			{
				ch1= read_lux_register();
				current_state = LUX_SENSOR_WAIT_FOR_I2C_COMMAND_COMPLETE;
				buffer_flag  = 0;
//				if (init_cycle_complete == 1)
//				{
					lux_value = get_lux_sensor_values(ch0, ch1);
					LOG_INFO("!!!!!CALCULATED LUXVAL = %lf", lux_value);
			//	}
				//init_cycle_complete = 1;
				LOG_INFO("In LUX_SENSOR_WAIT_FOR_I2C_WRITE_READ_COMPLETE STATEfor ch1 = %lf", ch1);
			}
		}
		else if ( ( event_name.EVENT_I2C_TRANSFER_ERROR ) )
		{
			event_name.EVENT_I2C_TRANSFER_COMPLETE = 0;
			LOG_DEBUG("Error at Write Complete, not done\n");
			current_state = LUX_SENSOR_WAIT_FOR_POWER_UP;
		}
		break;

	default:
		LOG_INFO("UNKNOWN STATE");
		break;
	}

	LOG_INFO("***exited state machine***");
}




void ps_store_sensor_data()
{
	LOG_INFO(" I2C working Luxval = : %lf",lux_value);
	uint16_t new_lux_val = (uint16_t)(lux_value * 100);
	LOG_INFO("New Luxval = : %d",new_lux_val);

	if (new_lux_val >= max_lux_val)
	{
		max_lux_val = new_lux_val;
		gecko_store_persistent_data(LUX_KEY, new_lux_val);
		char max_val[20];
		sprintf(max_val, "Lux Max: %f",(float)(max_lux_val)/100.0);
		displayPrintf(DISPLAY_ROW_PASSKEY, max_val);
	}

	publish_data(mesh_generic_state_level, lux_value*100, MESH_GENERIC_LEVEL_CLIENT_MODEL_ID );
	LOG_INFO("Published: LEVEL_VAL ");
	if (new_lux_val >= 25000)
	{
		publish_data(mesh_generic_request_on_off, MESH_GENERIC_ON_OFF_STATE_ON, MESH_GENERIC_ON_OFF_CLIENT_MODEL_ID );
		LOG_INFO("Published: ON ");
	}
	else
	{
		publish_data(mesh_generic_request_on_off, MESH_GENERIC_ON_OFF_STATE_OFF, MESH_GENERIC_ON_OFF_CLIENT_MODEL_ID );
		LOG_INFO("Published: OFF ");
	}
	//mesh_generic_state_level
	//mesh_generic_request_on_off
	//MESH_GENERIC_ON_OFF_CLIENT_MODEL_ID
	//MESH_GENERIC_LEVEL_CLIENT_MODEL_ID
}

