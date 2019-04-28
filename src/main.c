/**
 * This file initializes all the bluetooth and necessary system routines
 * for Blue Gecko Board. This is done as a part of Final Project for
 * ECEN 5823 IoT Embedded Firmware Course taught at the University of Colorado Boulder
 *
 * Author	:  Tanmay Chaturvedi
 * Dates 	:  5th April - 27th April
 * Reference:  Silicon Labs BT Mesh Switch Example

 * evt scheduler in gecko_main.c
 */
#include <stdbool.h>
#include "native_gecko.h"
#include "log.h"
#include <stdlib.h>
#include <stdio.h>

#include "i2c.h"
#include "src/mydisplay.h"
#include "src/gpio.h"
#include "gecko_main.h"
#include "src/event_scheduler.h"
#include "src/letimer0.h"


extern void gecko_main_init();
bool mesh_bgapi_listener(struct gecko_cmd_packet *evt);
extern void handle_gecko_event(uint32_t evt_id, struct gecko_cmd_packet *evt);



int main(void)
{

	// Initialize stack
	gecko_main_init();

	logInit();

	gpioInit();

	displayInit();

	LETIMER0_En_CLK_Tree();
	LETIMER0_Config();
	LETIMER0_SetInterrupt();

	load_power_on();
	I2C_init();

	//for lux sensor
	I2C_send_command(LUX_SENSOR_ADDR, LUX_COMMAND_BIT | LUX_CONTROL_REG, I2C_FLAG_WRITE_WRITE , LUX_POWER_ON);	//Power On.


	/* Infinite loop */
	while (1) {
		struct gecko_cmd_packet *evt = gecko_wait_event();
		bool pass = mesh_bgapi_listener(evt);
		if (pass) {
			handle_gecko_event(BGLIB_MSG_ID(evt->header), evt);
		}
	};
}


//handle_gecko_event() definition in gecko_main.c
