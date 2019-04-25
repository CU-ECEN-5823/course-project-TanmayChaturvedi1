/*
 * gecko_main.h
 *
 *  Created on: 20 Apr 2019
 *      Author: TanmayC
 */

#ifndef GECKO_MAIN_H_
#define GECKO_MAIN_H_


/* Bluetooth stack headers */
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

/* ***************************************************
 * MACROS (sourced from Silicon Labs Example)
 *
 * Timer handles defines.
 ****************************************************/
#define TIMER_ID_RESTART          78
#define TIMER_ID_FACTORY_RESET    77
#define TIMER_ID_PROVISIONING     66
#define TIMER_ID_RETRANS          10
#define TIMER_ID_FRIEND_FIND      20
#define TIMER_ID_NODE_CONFIGURED  30
#define	LUX_SENSOR_DATA			  40
#define	I2C_INIT_TIMER_EXPIRE	  99


/* ***************************************************
 * FUNCTION PROTOTYPES
 *
 ****************************************************/
void initiate_factory_reset(void);
void set_device_name(bd_addr *pAddr);
void publish_button_state(int retrans);
void publish_data(mesh_generic_request_t kind_type, uint16_t data, uint16_t model_identifier );
static void init_models(void);
void lpn_init(void);
void lpn_deinit(void);
static void client_request_cb(uint16_t model_id,
                          uint16_t element_index,
                          uint16_t client_addr,
                          uint16_t server_addr,
                          uint16_t appkey_index,
                          const struct mesh_generic_request *request,
                          uint32_t transition_ms,
                          uint16_t delay_ms,
                          uint8_t request_flags);

void mesh_lib_generic_server_event_handler(struct gecko_cmd_packet *evt);
static errorcode_t onoff_update_and_publish(uint16_t element_index,
                                            uint32_t remaining_ms);
static void state_changed_cb(uint16_t model_id,
                         uint16_t element_index,
                         const struct mesh_generic_state *current,
                         const struct mesh_generic_state *target,
                         uint32_t remaining_ms);

static errorcode_t onoff_update(uint16_t element_index, uint32_t remaining_ms);
uint16_t gecko_retrieve_persistent_data(uint16_t storage_key);
void gecko_store_persistent_data(uint16_t storage_key, uint16_t data);

#endif /* GECKO_MAIN_H_ */
