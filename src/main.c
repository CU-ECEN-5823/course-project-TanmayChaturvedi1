/* Modified by Tanmay Chaturvedi
 * Date: 3 April 2019
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

  //GPIO_PinModeSet( gpioPortD, 15, gpioModePushPull, 1 );
    I2C_init();
    /*//for lux sensor
    I2C_send_command(LUX_SENSOR_ADDR, LUX_COMMAND_BIT | LUX_CONTROL_REG, I2C_FLAG_WRITE_WRITE , LUX_POWER_ON);	//Power On.*/
   // I2C_read_byte(LUX_SENSOR_ADDR,LUX_COMMAND_BIT | LUX_CONTROL_REG , I2C_FLAG_WRITE_READ);
extern double lux_value;
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
