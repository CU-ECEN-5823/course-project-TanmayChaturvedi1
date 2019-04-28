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

/*Bluetooth Stack Includes*/
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
#define	LUX_THRESHOLD	(500.0)


/* @State machine for APDS 9301 Lux Sensor
 * Called every 10seconds to acquire lux data
 *
 * @param	ext_signal:  external signal generated either when I2C transfer complete
 * or COMP1 interrupt which occurs after generating noin-blocking time delay
 * @return	none
 */
void acquire_lux_data(uint32_t ext_signal);

/* @brief Store data in NVM Persistent memory and publish data
 * Checks if the new lux value > max lux val. If yes,store the new max value to PS and publish,
 * else, just publish using publish_data();
 *
 * @param	none
 * @return	none
 */
void ps_store_sensor_data();

/*States*/
enum lux_sensor_state {
	LUX_SENSOR_WAIT_FOR_POWER_UP = 0,
	LUX_SENSOR_WAIT_FOR_I2C_COMMAND_COMPLETE,
	LUX_SENSOR_WAIT_FOR_I2C_WRITE_READ_COMPLETE,
	LUX_SENSOR_WAIT_FOR_I2C_READ_COMPLETE,
};

/*Events*/
typedef struct events {
	bool	EVENT_NONE;
	bool	EVENT_I2C_TRANSFER_COMPLETE;
	bool	EVENT_I2C_TRANSFER_ERROR;
	bool	EVENT_INITIATE_STATE_MACHINE;
	bool	EVENT_SETUP_TIMER_EXPIRED;	//for COMP1 Interrupt
}event_name_t;

event_name_t event_name ;

uint8_t buffer_flag;
uint8_t command_flag;
uint8_t init_cycle_complete;

#define	START_LUX_STATE_MACHINE	(uint32_t)(0x01)
#define TRANSFER_COMPLETE		(uint32_t)(0x03)
#define COMP1_EVENT				(uint32_t)(0x04)

#endif /* SRC_EVENT_SCHEDULER_H_ */
