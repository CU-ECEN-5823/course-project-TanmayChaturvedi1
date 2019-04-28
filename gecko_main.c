/***************************************************************************//**
 * This file contains all the bluetooth-based handlers and other functionalities
 * to implement Low Power Mesh Node. This is done as a part of Final Project for
 * ECEN 5823 IoT Embedded Firmware Course taught at the University of Colorado Boulder
 *
 * Author	:  Tanmay Chaturvedi
 * Dates 	:  5th April - 27th April
 * Reference:  Silicon Labs BT Mesh Switch Example

 OLD DESCRIPTION-->
 * @file
 * @brief Silicon Labs BT Mesh Empty Example Project
 * This example demonstrates the bare minimum needed for a Blue Gecko BT Mesh C application.
 * The application starts unprovisioned Beaconing after boot
 *******************************************************************************
 * # License
 * <b>Copyright 2018 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 * Last Modified by Tanmay Chaturvedi
 * Date: 27 April 2019
 *
 ******************************************************************************/
/* C Standard Library headers */
#include <stdlib.h>
#include <stdio.h>

/* Board headers */
#include "init_mcu.h"
#include "init_board.h"
#include "init_app.h"
#include "ble-configuration.h"
#include "board_features.h"

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

/* Libraries containing default Gecko configuration values */
#include "em_emu.h"
#include "em_cmu.h"
#include <em_gpio.h>

/* Device initialization header */
#include "hal-config.h"

#if defined(HAL_CONFIG)
#include "bsphalconfig.h"
#else
#include "bspconfig.h"
#endif
#include "src/ble_mesh_device_type.h"
#include "src/mydisplay.h"
#include "src/gpio.h"
#include "src/log.h"
#include "i2c.h"
#include "gecko_main.h"
#include "src/gecko_ble_errors.h"
#include "src/event_scheduler.h"



/***********************************************************************************************//**
 * @addtogroup Application
 * @{
 **************************************************************************************************/

/***********************************************************************************************//**
 * @addtogroup app
 * @{
 **************************************************************************************************/

// bluetooth stack heap
#define MAX_CONNECTIONS 2

uint8_t bluetooth_stack_heap[DEFAULT_BLUETOOTH_HEAP(MAX_CONNECTIONS) + BTMESH_HEAP_SIZE + 1760];

// Bluetooth advertisement set configuration
//
// At minimum the following is required:
// * One advertisement set for Bluetooth LE stack (handle number 0)
// * One advertisement set for Mesh data (handle number 1)
// * One advertisement set for Mesh unprovisioned beacons (handle number 2)
// * One advertisement set for Mesh unprovisioned URI (handle number 3)
// * N advertisement sets for Mesh GATT service advertisements
// (one for each network key, handle numbers 4 .. N+3)
//
#define MAX_ADVERTISERS (4 + MESH_CFG_MAX_NETKEYS)
#define	LUX_KEY			(0x4000)

static gecko_bluetooth_ll_priorities linklayer_priorities = GECKO_BLUETOOTH_PRIORITIES_DEFAULT;

// bluetooth stack configuration
extern const struct bg_gattdb_def bg_gattdb_data;

double lux_value = 0;
// Flag for indicating DFU Reset must be performed
uint8_t boot_to_dfu = 0;

const gecko_configuration_t config =
{
		.sleep.flags = SLEEP_FLAGS_DEEP_SLEEP_ENABLE,
		.bluetooth.max_connections = MAX_CONNECTIONS,
		.bluetooth.max_advertisers = MAX_ADVERTISERS,
		.bluetooth.heap = bluetooth_stack_heap,
		.bluetooth.heap_size = sizeof(bluetooth_stack_heap) - BTMESH_HEAP_SIZE,
		.bluetooth.sleep_clock_accuracy = 100,
		.bluetooth.linklayer_priorities = &linklayer_priorities,
		.gattdb = &bg_gattdb_data,
		.btmesh_heap_size = BTMESH_HEAP_SIZE,
#if (HAL_PA_ENABLE)
		.pa.config_enable = 1, // Set this to be a valid PA config
#if defined(FEATURE_PA_INPUT_FROM_VBAT)
		.pa.input = GECKO_RADIO_PA_INPUT_VBAT, // Configure PA input to VBAT
#else
		.pa.input = GECKO_RADIO_PA_INPUT_DCDC,
#endif // defined(FEATURE_PA_INPUT_FROM_VBAT)
#endif // (HAL_PA_ENABLE)
		.max_timers = 16,
};

void handle_gecko_event(uint32_t evt_id, struct gecko_cmd_packet *evt);
void mesh_native_bgapi_init(void);
bool mesh_bgapi_listener(struct gecko_cmd_packet *evt);

/**
 * See light switch app.c file definition
 */
void gecko_bgapi_classes_init_server_friend(void)
{
	gecko_bgapi_class_dfu_init();
	gecko_bgapi_class_system_init();
	gecko_bgapi_class_le_gap_init();
	gecko_bgapi_class_le_connection_init();
	//gecko_bgapi_class_gatt_init();
	gecko_bgapi_class_gatt_server_init();
	gecko_bgapi_class_hardware_init();
	gecko_bgapi_class_flash_init();
	gecko_bgapi_class_test_init();
	//gecko_bgapi_class_sm_init();
	//mesh_native_bgapi_init();
	gecko_bgapi_class_mesh_node_init();
	//gecko_bgapi_class_mesh_prov_init();
	gecko_bgapi_class_mesh_proxy_init();
	gecko_bgapi_class_mesh_proxy_server_init();
	//gecko_bgapi_class_mesh_proxy_client_init();
	//gecko_bgapi_class_mesh_generic_client_init();
	gecko_bgapi_class_mesh_generic_server_init();
	//gecko_bgapi_class_mesh_vendor_model_init();
	//gecko_bgapi_class_mesh_health_client_init();
	//gecko_bgapi_class_mesh_health_server_init();
	//gecko_bgapi_class_mesh_test_init();
	//gecko_bgapi_class_mesh_lpn_init();
	gecko_bgapi_class_mesh_friend_init();
}



/**
 * See main function list in soc-btmesh-switch project file
 */
void gecko_bgapi_classes_init_client_lpn(void)
{
	gecko_bgapi_class_dfu_init();
	gecko_bgapi_class_system_init();
	gecko_bgapi_class_le_gap_init();
	gecko_bgapi_class_le_connection_init();
	//gecko_bgapi_class_gatt_init();
	gecko_bgapi_class_gatt_server_init();
	gecko_bgapi_class_hardware_init();
	gecko_bgapi_class_flash_init();
	gecko_bgapi_class_test_init();
	//gecko_bgapi_class_sm_init();
	//mesh_native_bgapi_init();
	gecko_bgapi_class_mesh_node_init();
	//gecko_bgapi_class_mesh_prov_init();
	gecko_bgapi_class_mesh_proxy_init();
	gecko_bgapi_class_mesh_proxy_server_init();
	//gecko_bgapi_class_mesh_proxy_client_init();
	gecko_bgapi_class_mesh_generic_client_init();
	//gecko_bgapi_class_mesh_generic_server_init();
	//gecko_bgapi_class_mesh_vendor_model_init();
	//gecko_bgapi_class_mesh_health_client_init();
	//gecko_bgapi_class_mesh_health_server_init();
	//gecko_bgapi_class_mesh_test_init();
	gecko_bgapi_class_mesh_lpn_init();
	//gecko_bgapi_class_mesh_friend_init();

}


void gecko_main_init()
{
	// Initialize device
	initMcu();

	// Initialize board
	initBoard();

	// Initialize application
	initApp();

	// Minimize advertisement latency by allowing the advertiser to always
	// interrupt the scanner.
	linklayer_priorities.scan_max = linklayer_priorities.adv_min + 1;

	gecko_stack_init(&config);

#if DEVICE_IS_ONOFF_PUBLISHER

	gecko_bgapi_classes_init_client_lpn();
#else
	gecko_bgapi_classes_init_server_friend();
#endif

	// Initialize coexistence interface. Parameters are taken from HAL config.
	gecko_initCoexHAL();

	//gecko_bgapi_classes_init();

	// Initialize coexistence interface. Parameters are taken from HAL config.
	gecko_initCoexHAL();

	RETARGET_SerialInit();
#if defined(_SILICON_LABS_32B_SERIES_1_CONFIG_3)
	/* xG13 devices have two RTCCs, one for the stack and another for the application.
	 * The clock for RTCC needs to be enabled in application code. In xG12 RTCC init
	 * is handled by the stack */
	CMU_ClockEnable(cmuClock_RTCC, true);
#endif

}



/* ***************************************************
 * GLOBAL VARIABLES
 *
 ****************************************************/
static uint8 request_count;

// handle of the last opened LE connection
static uint8 conn_handle = 0xFF;

// For indexing elements of the node
static uint16 _elem_index = 0xffff;

// number of active Bluetooth connections
static uint8 num_connections = 0;

// Flag for indicating that lpn feature is active
static uint8 lpn_active = 0;

// transaction identifier
static uint8 trid = 0;
static uint8 trid1 = 0;

// current position of the switch
volatile uint8 switch_pos = 0;

// For indexing elements of the node
static uint16 _primary_elem_index = 0xffff;

// For storing max data in PS
uint16_t max_lux_val = 0;

#if DEVICE_IS_ONOFF_PUBLISHER

/*Event Handler for BL Mesh and BLE Stack
 * for Publisher
 *
 * @param	evt_id:	Incoming event ID
 * @param	*evt:	Pointer to event
 * @return	none
 */
void handle_gecko_event(uint32_t evt_id, struct gecko_cmd_packet *evt)
{
	CORE_DECLARE_IRQ_STATE;
	uint16 ret_status;
	char buf[30];

	if (NULL == evt)
	{
		return;
	}
	switch (evt_id) {


	/* EVENT->
	 * System Initialization
	 * */
	case gecko_evt_system_boot_id:
		LOG_INFO("gecko_evt_system_boot_id");
		//Initialize Mesh stack in Node operation mode, wait for initialized event and
		//Check if either PB0 or PB1 is pressed at Startup
		//Should initiate factory reset at Startup
		if (GPIO_PinInGet(BSP_BUTTON0_PORT, BSP_BUTTON0_PIN) == 0 || GPIO_PinInGet(BSP_BUTTON1_PORT, BSP_BUTTON1_PIN) == 0)
		{
			//Initiate Factory Reset
			initiate_factory_reset();
			if (conn_handle != 0xFF)
			{
				gecko_cmd_le_connection_close(conn_handle);
			}
#if ECEN5823_INCLUDE_DISPLAY_SUPPORT
			displayPrintf(DISPLAY_ROW_ACTION,"Factory Reset");
#endif

		}

		else	//if no button press detected
		{
			LOG_INFO("no button press detected");
			//Get BT address
			struct gecko_msg_system_get_bt_address_rsp_t *bt_addr = gecko_cmd_system_get_bt_address();

			set_device_name(&bt_addr->address);
			BTSTACK_CHECK_RESPONSE(gecko_cmd_mesh_node_init());
			uint16_t old_lux_val = gecko_retrieve_persistent_data(LUX_KEY);
			char old_val[20];
			sprintf(old_val, "Lux old: %f",(float)(old_lux_val)/10.0);
#if ECEN5823_INCLUDE_DISPLAY_SUPPORT
			displayPrintf(DISPLAY_ROW_PASSKEY, old_val);
#endif
		}

		break;

	/* EVENT->
	 * Software ticks timer expires
	 * */
	case gecko_evt_hardware_soft_timer_id:
		switch(evt->data.evt_hardware_soft_timer.handle)
		{
		case TIMER_ID_FACTORY_RESET:
			gecko_cmd_system_reset(0);
#if ECEN5823_INCLUDE_DISPLAY_SUPPORT
			displayPrintf(DISPLAY_ROW_NAME,"");
#endif
			break;

		case LOG_TIMER:
			msec += 10;
			break;

		case LUX_SENSOR_DATA:
			load_power_on();
			I2C_send_command(LUX_SENSOR_ADDR, LUX_COMMAND_BIT | LUX_CONTROL_REG, I2C_FLAG_WRITE_WRITE , LUX_POWER_ON);
			gecko_cmd_hardware_set_soft_timer(33000,FETCH_LUX,1 );
			break;

		case FETCH_LUX:
			LOG_INFO("LOAD POWER MANAGEMENT DONE..........");
			/**Critical Section Starts*/
			CORE_ENTER_CRITICAL( );
			event_name.EVENT_INITIATE_STATE_MACHINE = true;
			event_name.EVENT_I2C_TRANSFER_COMPLETE = false;
			event_name.EVENT_I2C_TRANSFER_ERROR = false;
			event_name.EVENT_SETUP_TIMER_EXPIRED = false;
			event_name.EVENT_NONE = false;
			/*Critical Section Ends*/
			CORE_EXIT_CRITICAL();
			acquire_lux_data(START_LUX_STATE_MACHINE);
			break;

		case UPDATE_DISPLAY:
			//LOG_INFO("display update");
			//displayUpdate();
			break;

		case LOAD_OFF:
			load_power_off();
			break;

		case TIMER_ID_NODE_CONFIGURED:
			LOG_INFO("Post Provision Timer Expired, Initiate LPN");
			lpn_init();
			break;

		case TIMER_ID_FRIEND_FIND:
		{
			LOG_INFO("trying to find friend...");
			BTSTACK_CHECK_RESPONSE(gecko_cmd_mesh_lpn_establish_friendship(0));
		}

		}
		break;


	/* EVENT->
	 * Node initialized
	 * */
	case gecko_evt_mesh_node_initialized_id:
		//if not provisioned, start provisioning
		if (!evt->data.evt_mesh_node_initialized.provisioned)
		{
			// The Node is now initialized, start unprovisioned Beaconing using PB-ADV and PB-GATT Bearers
			gecko_cmd_mesh_node_start_unprov_beaconing(0x3);
			LOG_INFO("Beaconing started, and initialized");
		}
		//if Provisioned
		if (evt->data.evt_mesh_node_initialized.provisioned)
		{
			_elem_index = 0;
			BTSTACK_CHECK_RESPONSE(gecko_cmd_mesh_generic_client_init());
			LOG_INFO("Provisioned already, initialize generic client models");
			LOG_INFO("Interrupt Enabled");
			// Initialize mesh lib, up to 8 models
			mesh_lib_init(malloc, free, 8);
			lpn_init();
		}
		break;


	/* EVENT->
	 * Provisioning Started
	 * */
	case gecko_evt_mesh_node_provisioning_started_id:
#if ECEN5823_INCLUDE_DISPLAY_SUPPORT
		displayPrintf(DISPLAY_ROW_ACTION,"Provisioning");
#endif
		break;

	/* EVENT->
	 * Provisioning Done!
	 * */
	case gecko_evt_mesh_node_provisioned_id:
#if ECEN5823_INCLUDE_DISPLAY_SUPPORT
		displayPrintf(DISPLAY_ROW_ACTION,"Provisioned");
#endif
		// try to initialize lpn after 30 seconds, if no configuration messages come
		//		ret_status = gecko_cmd_hardware_set_soft_timer((30000),
		//												 TIMER_ID_NODE_CONFIGURED,
		//												 1)->result;

		break;


	/* EVENT->
	 * Proviosioning Failed
	 * */
	case gecko_evt_mesh_node_provisioning_failed_id:
#if ECEN5823_INCLUDE_DISPLAY_SUPPORT
		displayPrintf(DISPLAY_ROW_ACTION,"Provisioning Failed");
#endif
		break;


	/* EVENT->
	 * Gatt Connection Opened
	 * */
	case gecko_evt_le_connection_opened_id:
		//Store connection handle information
		conn_handle = evt->data.evt_le_connection_opened.connection;
#if ECEN5823_INCLUDE_DISPLAY_SUPPORT
		displayPrintf(DISPLAY_ROW_CONNECTION,"Connected");
#endif
		break;

	/* EVENT->
	 * Gatt connection Closed
	 * */
	case gecko_evt_le_connection_closed_id:
		/* Check if need to boot to dfu mode */
		if (boot_to_dfu) {
			/* Enter to DFU OTA mode */
			gecko_cmd_system_reset(2);
		}
#if ECEN5823_INCLUDE_DISPLAY_SUPPORT
		displayPrintf(DISPLAY_ROW_CONNECTION, "");
#endif
		LOG_INFO("Connection Closed");

		break;


	/* EVENT->
	 * External Signal to initiate/continue I2C state machine
	 * */
	case( gecko_evt_system_external_signal_id ):
		LOG_INFO("Begin: gecko_evt_system_external_signal_id\n");
		acquire_lux_data(evt->data.evt_system_external_signal.extsignals);
		break;

	/* EVENT->
	 * Server user write request
	 * */
	case gecko_evt_gatt_server_user_write_request_id:
		if (evt->data.evt_gatt_server_user_write_request.characteristic == gattdb_ota_control) {
			/* Set flag to enter to OTA mode */
			boot_to_dfu = 1;
			/* Send response to Write Request */
			gecko_cmd_gatt_server_send_user_write_response(
					evt->data.evt_gatt_server_user_write_request.connection,
					gattdb_ota_control,
					bg_err_success);

			/* Close connection to enter to DFU OTA mode */
			gecko_cmd_le_connection_close(evt->data.evt_gatt_server_user_write_request.connection);
		}
		break;


	/* EVENT->
	 * Friendship Successfully Established
	 * */
	case gecko_evt_mesh_lpn_friendship_established_id:
		LOG_INFO("friendship established");
#if ECEN5823_INCLUDE_DISPLAY_SUPPORT
		displayPrintf(DISPLAY_ROW_CONNECTION, "LPN with friend");
#endif
		//Only when friendship successful, enable MQ2 GPIO interrupt pins and repeated timer for lux data acquisition,
		gpio_set_interrupt();
		gecko_cmd_hardware_set_soft_timer(10* 32768,LUX_SENSOR_DATA,0 );

		break;


	/* EVENT->
	 * Friendship Failed
	 * */
	case gecko_evt_mesh_lpn_friendship_failed_id:
		LOG_INFO("friendship failed");
#if ECEN5823_INCLUDE_DISPLAY_SUPPORT
		displayPrintf(DISPLAY_ROW_CONNECTION, "No Friend");
#endif
		BTSTACK_CHECK_RESPONSE(gecko_cmd_hardware_set_soft_timer(2000, TIMER_ID_FRIEND_FIND, 1));
		break;


	/* EVENT->
	 * Friendship Terminated
	 * */
	case gecko_evt_mesh_lpn_friendship_terminated_id:
		printf("friendship terminated");
#if ECEN5823_INCLUDE_DISPLAY_SUPPORT
		displayPrintf(DISPLAY_ROW_CONNECTION, "");
#endif
		if (num_connections == 0) {
			// try again in 2 seconds
			BTSTACK_CHECK_RESPONSE(gecko_cmd_hardware_set_soft_timer(2000, TIMER_ID_FRIEND_FIND, 1));
		}
		break;


	/* EVENT->
	 * Node reset
	 * */
	case gecko_evt_mesh_node_reset_id:
		BTSTACK_CHECK_RESPONSE(gecko_cmd_flash_ps_erase_all());
		BTSTACK_CHECK_RESPONSE(gecko_cmd_hardware_set_soft_timer( 2 * 32768, TIMER_ID_FACTORY_RESET, 1));
		break;

	default:
		break;
}
}



#else

#endif


/* If any button press detected,
 * Clear the Persistent Storage Memory
 * Configure a timer to perform reset after 2 seconds
 *
 */
void initiate_factory_reset(void)
{
	//Clear PS
	BTSTACK_CHECK_RESPONSE(gecko_cmd_flash_ps_erase_all());
	// reboot after a small delay
	BTSTACK_CHECK_RESPONSE(gecko_cmd_hardware_set_soft_timer(2 * 32768, TIMER_ID_FACTORY_RESET, 1));
}



/* Set device name in the GATT database. A unique name is generated using
 * the two last bytes from the Bluetooth address of this device. Name is also
 * displayed on the LCD.
 *
 * @param[in] pAddr  Pointer to Bluetooth address.
 * [Source]: Silicon Labs Mesh Example
 */
void set_device_name(bd_addr *pAddr)
{
	char name[20];
	uint16 res;

	// create unique device name using the last two bytes of the Bluetooth address

#if DEVICE_IS_ONOFF_PUBLISHER
	sprintf(name, "ECENPub %02x:%02x", pAddr->addr[1], pAddr->addr[0]);
#else
	sprintf(name, "ECENSub %02x:%02x", pAddr->addr[1], pAddr->addr[0]);
#endif

	// write device name to the GATT database
	BTSTACK_CHECK_RESPONSE(gecko_cmd_gatt_server_write_attribute_value(gattdb_device_name, 0, strlen(name), (uint8 *)name));
	// show device name on the LCD
#if ECEN5823_INCLUDE_DISPLAY_SUPPORT
	displayPrintf(DISPLAY_ROW_NAME,name);
#endif
}


/* Publish Data to the Friend node based on different BL Mesh Model
 *
 * @param	kind_type:	Request Kind - Possible values -> mesh_generic_state_level, mesh_generic_request_on_off
 * @param	data:	Data to be published.
 * @param	model_identifier - Possible values -> MESH_GENERIC_ON_OFF_CLIENT_MODEL_ID,MESH_GENERIC_LEVEL_CLIENT_MODEL_ID
 * @return	none
 */
void publish_data(mesh_generic_request_t kind_type, uint16_t data, uint16_t model_identifier )
{
	int retrans = 0;
	int retrans1 = 0;
	uint16 resp, resp1;
	uint16 delay, delay1;
	const uint32 transtime = 0; /* using zero transition time by default */
	const uint32 transtime1 = 0; /* using zero transition time by default */
	delay = 0;
	delay1 = 0;
	// increment transaction ID for each request, unless it's a retransmission
	if (retrans == 0) {
		trid++;
	}
	if (retrans1 == 0) {
		trid1++;
	}

	if (model_identifier == MESH_GENERIC_ON_OFF_CLIENT_MODEL_ID )
	{
		LOG_INFO("In ON OFF CLIENT MODEL");
		struct mesh_generic_request req;
		req.kind = kind_type;
		req.on_off = data;
		LOG_INFO("req.on_off %d", req.on_off );
        displayPrintf(DISPLAY_ROW_ACTION,"BELOW THRESHOLD");
		delay = 2;
		resp = mesh_lib_generic_client_publish(
				model_identifier,
				0,
				trid,
				&req,
				2,   // transition time in ms
				delay,
				0     // flags
		);

		if (resp) {
			LOG_ERROR("gecko_cmd_mesh_generic_client_publish failed,code %x", resp);
		} else {
			LOG_INFO("request sent, trid = %u, delay = %d", trid, delay);
		}
	}

	if(model_identifier == MESH_GENERIC_LEVEL_CLIENT_MODEL_ID)
	{
		LOG_INFO("In LEVEL CLIENT MODEL");
		struct mesh_generic_state req1;
		req1.kind = kind_type;
		req1.level.level = data;
		LOG_INFO("req1.level.level %d", req1.level.level );


		char final_lux[30];
		sprintf(final_lux, "Lux now = %f",data/10.0);
#if ECEN5823_INCLUDE_DISPLAY_SUPPORT
		displayPrintf(DISPLAY_ROW_TEMPVALUE,final_lux);
#endif
		delay1 = 2;
		resp1 = mesh_lib_generic_client_publish(
				model_identifier,
				0,
				trid1,
				&req1,
				2,   // transition time in ms
				delay1,
				0     // flags
		);


		if (resp1) {
			LOG_ERROR("gecko_cmd_mesh_generic_client_publish failed,code %x", resp1);
		} else {
			LOG_INFO("request sent, trid = %u, delay = %d", trid1, delay1);
		}
	}

	else
	{

	}
}


/* {Deprecated (As of April 20, This function is no longer used, as publish-on-button-press not needed.
 * can be used to check if button presses/releases are published to subscriber}
 * Publishes the state of Button PB0 to the subscriber
 * @param	none
 * @return	none
 */
void publish_button_state(int button_state)
{
	int retrans = 0;
	uint16 resp;
	uint16 delay;
	struct mesh_generic_request req;
	const uint32 transtime = 0; /* using zero transition time by default */
	delay = 0;
	req.kind = mesh_generic_request_on_off;
	req.on_off = button_state ? MESH_GENERIC_ON_OFF_STATE_ON : MESH_GENERIC_ON_OFF_STATE_OFF;

	LOG_INFO("req.on_off %d", req.on_off );

	// increment transaction ID for each request, unless it's a retransmission
	if (retrans == 0) {
		trid++;
	}

	if (button_state == MESH_GENERIC_ON_OFF_STATE_ON)
	{

#if ECEN5823_INCLUDE_DISPLAY_SUPPORT
		displayPrintf(DISPLAY_ROW_TEMPVALUE,"Pressed");
#endif
	}
	if ( button_state == MESH_GENERIC_ON_OFF_STATE_OFF)
	{
#if ECEN5823_INCLUDE_DISPLAY_SUPPORT
		displayPrintf(DISPLAY_ROW_TEMPVALUE,"Released");
#endif
	}

	resp = mesh_lib_generic_client_publish(
			MESH_GENERIC_ON_OFF_CLIENT_MODEL_ID,
			0,
			trid,
			&req,
			transtime,   // transition time in ms
			delay,
			0     // flags
	);
	if (resp) {
		LOG_ERROR("gecko_cmd_mesh_generic_client_publish failed,code %x", resp);
	} else {
		LOG_INFO("request sent, trid = %u, delay = %d", trid, delay);
	}
}


struct reg {
	uint16_t model_id;
	uint16_t elem_index;
	union {
		struct {
			mesh_lib_generic_server_client_request_cb client_request_cb;
			mesh_lib_generic_server_change_cb state_changed_cb;
		} server;
		struct {
			mesh_lib_generic_client_server_response_cb server_response_cb;
		} client;
	};
};


/***************************************************************************//**
 * Initialize LPN functionality with configuration and friendship establishment.
 * Source: Silicon Labs Mesh Example
 ******************************************************************************/
void lpn_init(void)
{
	LOG_INFO("Establishing LPN");
	// Do not initialize LPN if lpn is currently active
	// or any GATT connection is opened
	if (lpn_active || num_connections)
	{
		return;
	}

	// Initialize LPN functionality.
	BTSTACK_CHECK_RESPONSE(gecko_cmd_mesh_lpn_init());
	lpn_active = 1;
	LOG_INFO("LPN initialized");
#if ECEN5823_INCLUDE_DISPLAY_SUPPORT
	displayPrintf(DISPLAY_ROW_CONNECTION,"LPN Init done");
#endif

	// Configure the lpn with following parameters:
	// - Minimum friend queue length = 2
	// - Poll timeout = 5 seconds
	BTSTACK_CHECK_RESPONSE(gecko_cmd_mesh_lpn_configure(2, 5 * 1000));
	LOG_INFO("trying to find friend...");
	BTSTACK_CHECK_RESPONSE(gecko_cmd_mesh_lpn_establish_friendship(0));
}



/***************************************************************************//**
 * Deinitialize LPN functionality.
 * Source: Silicon Labs Mesh Example
 ******************************************************************************/
void lpn_deinit(void)
{
	uint16 result;

	if (!lpn_active)
	{
		return; // lpn feature is currently inactive
	}

	BTSTACK_CHECK_RESPONSE(gecko_cmd_hardware_set_soft_timer(0, TIMER_ID_FRIEND_FIND, 1));

	// Terminate friendship if exist

	BTSTACK_CHECK_RESPONSE(gecko_cmd_mesh_lpn_terminate_friendship());
	BTSTACK_CHECK_RESPONSE(gecko_cmd_mesh_lpn_deinit());
	// turn off lpn feature
	lpn_active = 0;
	LOG_INFO("LPN deinitialized");
#if ECEN5823_INCLUDE_DISPLAY_SUPPORT
	displayPrintf(DISPLAY_ROW_CONNECTION,"LPN De-Init done");
#endif
}


/**
 * @brief Storage persistent data to Flash
 * @param uint16_t storage key - For lux sensor, enter "LUX_KEY" or 0x4000
 * @param uint16_t data to be stored in the PS
 * @return null
 */
void gecko_store_persistent_data(uint16_t storage_key, uint16_t data)
{
	int resp;
	uint8_t *ptr_data = &data;
	BTSTACK_CHECK_RESPONSE(gecko_cmd_flash_ps_save(storage_key, sizeof(data), ptr_data));
}

/**
 * @brief Retrieve persistent data from Flash
 *
 * @param uint16_t storage key - For lux sensor, enter "LUX_KEY" or 0x4000
 * @return uint16_t retrieved data
 */
uint16_t gecko_retrieve_persistent_data(uint16_t storage_key)
{
	uint16_t retrieved_data;
	struct gecko_msg_flash_ps_load_rsp_t *resp;
	resp = gecko_cmd_flash_ps_load(storage_key);
	if(resp->result){
		LOG_ERROR("gecko_retrieve_persistent_data failed, code %x", resp->result);
	}
	else{

		LOG_INFO("Data Retrieved in persistent memory");
		memcpy(&retrieved_data, &resp->value.data, resp->value.len);
		LOG_INFO("Data Retrieved = %d", retrieved_data);
	}
	return retrieved_data;
}


