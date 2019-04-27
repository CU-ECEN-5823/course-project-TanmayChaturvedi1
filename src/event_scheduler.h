/*
 * event_scheduler.h
 *
 *  Created on: 24 Apr 2019
 *      Author: TanmayC
 */

#ifndef SRC_EVENT_SCHEDULER_H_
#define SRC_EVENT_SCHEDULER_H_

#include "stdint.h"
#include "stdbool.h"
#include "em_core.h"
#include "i2c.h"
#include "src/log.h"
#include "display.h"
#include "mydisplay.h"
#include "gecko_main.h"
#include "src/nonblock_timerWaitUs.h"
#include "gpio.h"


#include "bg_types.h"
#include "native_gecko.h"
#include "gatt_db.h"
#include <gecko_configuration.h>
#include <mesh_sizes.h>
#include "mesh_generic_model_capi_types.h"
#include "mesh_lighting_model_capi_types.h"
#include "mesh_lib.h"
#include "native_gecko.h"
#include "retargetserial.h"

#define	LUX_KEY			(0x4000)

void acquire_lux_data(uint32_t ext_signal);
void ps_store_sensor_data();
//void gecko_external_signal();

enum lux_sensor_state {
	LUX_SENSOR_WAIT_FOR_POWER_UP = 0,
	LUX_SENSOR_WAIT_FOR_I2C_COMMAND_COMPLETE,
	LUX_SENSOR_WAIT_FOR_I2C_WRITE_READ_COMPLETE,
	LUX_SENSOR_WAIT_FOR_I2C_READ_COMPLETE,
};

 typedef struct events {
	bool	EVENT_NONE;
	bool	EVENT_I2C_TRANSFER_COMPLETE;
	bool	EVENT_I2C_TRANSFER_ERROR;
	bool	EVENT_INITIATE_STATE_MACHINE;
	bool	EVENT_SETUP_TIMER_EXPIRED;	//for COMP1 Interrupt
	bool	EVENT_TIMER_UNDERFLOW_INT;	//for COMP0 Underflow interrupt
}event_name_t;

event_name_t event_name ;

uint8_t buffer_flag;
uint8_t command_flag;
uint8_t init_cycle_complete;

#define	START_LUX_STATE_MACHINE	(uint32_t)(0x01)
//#define LETIMER_UF_EVENT		(uint32_t)(0x01)
#define TRANSFER_COMPLETE		(uint32_t)(0x03)
#define COMP1_EVENT				(uint32_t)(0x04)

#endif /* SRC_EVENT_SCHEDULER_H_ */
