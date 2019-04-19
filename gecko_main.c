/***************************************************************************//**
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
 * Modified by Tanmay Chaturvedi
 * Date: 3 April 2019
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

static gecko_bluetooth_ll_priorities linklayer_priorities = GECKO_BLUETOOTH_PRIORITIES_DEFAULT;

// bluetooth stack configuration
extern const struct bg_gattdb_def bg_gattdb_data;

// Flag for indicating DFU Reset must be performed
uint8_t boot_to_dfu = 0;

const gecko_configuration_t config =
{
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



/* ***************************************************
 * FUNCTION PROTOTYPES
 *
 ****************************************************/
void initiate_factory_reset(void);
void set_device_name(bd_addr *pAddr);
void publish_button_state(int retrans);
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

// current position of the switch
volatile uint8 switch_pos = 0;

// For indexing elements of the node
static uint16 _primary_elem_index = 0xffff;

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
  uint16 ret_status;
  char buf[30];

  if (NULL == evt)
  {
	return;
  }
  switch (evt_id) {
    case gecko_evt_system_boot_id:
    	LOG_INFO("gecko_evt_system_boot_id");
    // Initialize Mesh stack in Node operation mode, wait for initialized event and
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
		ret_status = gecko_cmd_mesh_node_init()->result;
	}
      break;

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

    	case UPDATE_DISPLAY:
    		//LOG_INFO("display update");
    		displayUpdate();
    		break;

    	case TIMER_ID_NODE_CONFIGURED:
    		LOG_INFO("Post Provision Timer Expired, Initiate LPN");
    		lpn_init();
    		break;


        case TIMER_ID_FRIEND_FIND:
        {
          LOG_INFO("trying to find friend...");
          ret_status = gecko_cmd_mesh_lpn_establish_friendship(0)->result;

          if (ret_status != 0) {
            LOG_INFO("ret.code %x", ret_status);
          }
        }

    	}
    	break;




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
    	// Initialize generic client models
    		ret_status = gecko_cmd_mesh_generic_client_init()->result;
    		LOG_INFO("Provisioned already, initialize generic client models");
    	      if (ret_status)
    	      {
    	            LOG_INFO("mesh_generic_client_init failed, code 0x%x", ret_status);
    	      }
    	// Set GPIO Interrupt

    	      LOG_INFO("Interrupt Enabled");
    	// Initialize mesh lib, up to 8 models
    	      mesh_lib_init(malloc, free, 8);
    	      gpio_set_interrupt();
    	      lpn_init();
    	}

    	break;

    case gecko_evt_mesh_node_provisioning_started_id:
#if ECEN5823_INCLUDE_DISPLAY_SUPPORT
		displayPrintf(DISPLAY_ROW_ACTION,"Provisioning");
#endif
		break;

    case gecko_evt_mesh_node_provisioned_id:
#if ECEN5823_INCLUDE_DISPLAY_SUPPORT
		displayPrintf(DISPLAY_ROW_ACTION,"Provisioned");
#endif
	// try to initialize lpn after 30 seconds, if no configuration messages come
//		ret_status = gecko_cmd_hardware_set_soft_timer((30000),
//												 TIMER_ID_NODE_CONFIGURED,
//												 1)->result;

		break;

    case gecko_evt_mesh_node_provisioning_failed_id:
#if ECEN5823_INCLUDE_DISPLAY_SUPPORT
		displayPrintf(DISPLAY_ROW_ACTION,"Provisioning Failed");
#endif
		break;


    case gecko_evt_le_connection_opened_id:
    	//Store connection handle information
    	conn_handle = evt->data.evt_le_connection_opened.connection;
#if ECEN5823_INCLUDE_DISPLAY_SUPPORT
		displayPrintf(DISPLAY_ROW_CONNECTION,"Connected");
#endif
    	break;

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


    case gecko_evt_system_external_signal_id:
	if(evt->data.evt_system_external_signal.extsignals & PB0_STATE)
	{
		ext_sig_event &= ~(PB0_STATE);

		if (GPIO_PinInGet(gpioPortF,6))
		{
#if ECEN5823_INCLUDE_DISPLAY_SUPPORT
	displayPrintf(DISPLAY_ROW_ACTION, "Button Released");
#endif
		LOG_INFO("Publish : Button Released");
		publish_button_state(MESH_GENERIC_ON_OFF_STATE_OFF);
		break;
		}

		if (GPIO_PinInGet(gpioPortF,6)==0)
		{
		//ext_sig_event &= ~(PB0_RELEASED);
#if ECEN5823_INCLUDE_DISPLAY_SUPPORT
	displayPrintf(DISPLAY_ROW_ACTION, "Button Pressed");
#endif
		publish_button_state(MESH_GENERIC_ON_OFF_STATE_ON);
		LOG_INFO("Publish : Button Pressed");
		break;
		}

	}
	break;


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


    case gecko_evt_mesh_lpn_friendship_established_id:
    	LOG_INFO("friendship established");
#if ECEN5823_INCLUDE_DISPLAY_SUPPORT
	displayPrintf(DISPLAY_ROW_CONNECTION, "LPN with friend");
#endif
         break;

       case gecko_evt_mesh_lpn_friendship_failed_id:
         LOG_INFO("friendship failed");
#if ECEN5823_INCLUDE_DISPLAY_SUPPORT
	displayPrintf(DISPLAY_ROW_CONNECTION, "No Friend");
#endif        // try again in 2 seconds

		ret_status = gecko_cmd_hardware_set_soft_timer((2000),
                                                    TIMER_ID_FRIEND_FIND,
                                                    1)->result;
         if (ret_status)
         {
           LOG_INFO("timer failure?!  %x", ret_status);
         }
         break;

       case gecko_evt_mesh_lpn_friendship_terminated_id:
         printf("friendship terminated");
#if ECEN5823_INCLUDE_DISPLAY_SUPPORT
	displayPrintf(DISPLAY_ROW_CONNECTION, "");
#endif
	if (num_connections == 0) {
           // try again in 2 seconds
           ret_status = gecko_cmd_hardware_set_soft_timer(2000,
                                                      TIMER_ID_FRIEND_FIND,
                                                      1)->result;
           if (ret_status) {
             LOG_INFO("timer failure?!  %x", ret_status);
           }
         }
         break;


    case gecko_evt_mesh_node_reset_id:
    	gecko_cmd_flash_ps_erase_all();
    	gecko_cmd_hardware_set_soft_timer( 2 * 32768, TIMER_ID_FACTORY_RESET, 1);
    	break;

    default:
      break;
  }
}



#else

/*Event Handler for BL Mesh and BLE Stack
 * for Subscriber
 *
 * @param	evt_id:	Incoming event ID
 * @param	*evt:	Pointer to event
 * @return	none
 */
void handle_gecko_event(uint32_t evt_id, struct gecko_cmd_packet *evt)
{
  uint16 ret_status;
  char buf[30];

  if (NULL == evt)
  {
	return;
  }
  switch (evt_id) {
    case gecko_evt_system_boot_id:
    // Initialize Mesh stack in Node operation mode, wait for initialized event
	//Check if either PB0 or PB1 is pressed at Startup
	//Should initiate factory reset at Startup
	if (GPIO_PinInGet(BSP_BUTTON0_PORT, BSP_BUTTON0_PIN) == 0 || GPIO_PinInGet(BSP_BUTTON1_PORT, BSP_BUTTON1_PIN) == 0)
	{
		//Initiate Factory Reset
		initiate_factory_reset();
#if ECEN5823_INCLUDE_DISPLAY_SUPPORT
		displayPrintf(DISPLAY_ROW_ACTION,"Factory Reset");
#endif

	}

	else	//if no button press detected
	{
		//Get BT address
		struct gecko_msg_system_get_bt_address_rsp_t *bt_addr = gecko_cmd_system_get_bt_address();
		set_device_name(&bt_addr->address);
		ret_status = gecko_cmd_mesh_node_init()->result;
	}
      break;

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

    	case UPDATE_DISPLAY:
    		LOG_INFO("display update");
    		displayUpdate();
    		break;

    	}
    	break;



    case gecko_evt_mesh_node_initialized_id:
    	//if not provisioned, start provisioning
    	if (!evt->data.evt_mesh_node_initialized.provisioned)
    	{
        // The Node is now initialized, start unprovisioned Beaconing using PB-ADV and PB-GATT Bearers
    		gecko_cmd_mesh_node_start_unprov_beaconing(0x3);
    	}

    	//if Provisioned
    	if (evt->data.evt_mesh_node_initialized.provisioned)
    	{
    	// Initialize generic server models
    		ret_status = gecko_cmd_mesh_generic_server_init()->result;
    	      if (ret_status)
    	      {
    	    	  LOG_INFO("mesh_generic_server_init failed, code 0x%x", ret_status);
    	      }
    	// Initialize mesh lib, up to 9 models
    	      mesh_lib_init(malloc, free, 9);
    	      init_models();
    	      _primary_elem_index = 0;
    	      onoff_update_and_publish(_primary_elem_index, 0);
    	     // onoff_update(_primary_elem_index, 0);


    	}
    	break;

    case gecko_evt_mesh_node_provisioning_started_id:
#if ECEN5823_INCLUDE_DISPLAY_SUPPORT
		displayPrintf(DISPLAY_ROW_ACTION,"Provisioning");
#endif
		break;

    case gecko_evt_mesh_node_provisioned_id:
#if ECEN5823_INCLUDE_DISPLAY_SUPPORT
		displayPrintf(DISPLAY_ROW_ACTION,"Provisioned");
#endif
		break;

    case gecko_evt_mesh_node_provisioning_failed_id:
#if ECEN5823_INCLUDE_DISPLAY_SUPPORT
		displayPrintf(DISPLAY_ROW_ACTION,"Provisioning Failed");
#endif
		break;


    case gecko_evt_mesh_generic_server_client_request_id:
      // pass the server client request event to mesh lib handler that will invoke
      // the callback functions registered by application
      mesh_lib_generic_server_event_handler(evt);
      break;


    case gecko_evt_mesh_generic_server_state_changed_id:

      // uncomment following line to get debug prints for each server state changed event
      //server_state_changed(&(evt->data.evt_mesh_generic_server_state_changed));

      // pass the server state changed event to mesh lib handler that will invoke
      // the callback functions registered by application
      mesh_lib_generic_server_event_handler(evt);
      break;


    case gecko_evt_le_connection_opened_id:
    	//Store connection handle information
    	conn_handle = evt->data.evt_le_connection_opened.connection;
#if ECEN5823_INCLUDE_DISPLAY_SUPPORT
		displayPrintf(DISPLAY_ROW_CONNECTION,"Connected");
#endif
    	break;


    case gecko_evt_le_connection_closed_id:
      /* Check if need to boot to dfu mode */
      if (boot_to_dfu) {
        /* Enter to DFU OTA mode */
        gecko_cmd_system_reset(2);
      }
      break;



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

    case gecko_evt_mesh_node_reset_id:
    	gecko_cmd_flash_ps_erase_all();
    	gecko_cmd_hardware_set_soft_timer( 2 * 32768, TIMER_ID_FACTORY_RESET, 1);
    	break;
    default:
      break;
  }
}





#endif

/* If any button press detected,
 * Clear the Persistent Storage Memory
 * Configure a timer to perform reset after 2 seconds
 *
 */
void initiate_factory_reset(void)
{
	/*Need to acquire previous LE connection handle*/

	//Clear PS
	gecko_cmd_flash_ps_erase_all();
	// reboot after a small delay
	gecko_cmd_hardware_set_soft_timer(2 * 32768, TIMER_ID_FACTORY_RESET, 1);
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

  //printf("Device name: '%s'\r\n", name);

  // write device name to the GATT database
  res = gecko_cmd_gatt_server_write_attribute_value(gattdb_device_name, 0, strlen(name), (uint8 *)name)->result;
  if (res) {
   LOG_INFO("gecko_cmd_gatt_server_write_attribute_value() failed, code %x\r\n", res);
  }

  // show device name on the LCD
#if ECEN5823_INCLUDE_DISPLAY_SUPPORT
  	  displayPrintf(DISPLAY_ROW_NAME,name);
#endif
}


/*
 * Publishes the state of Button PB0 to the subscriber
 * @param	none
 * @return	none
 */
void publish_button_state(int button_state)
{
	  int retrans = 0;
	  uint16 resp;
	  uint16 delay;
//	  struct mesh_generic_request req;
	  struct mesh_generic_state req;
	  const uint32 transtime = 0; /* using zero transition time by default */
	  delay = 0;

	  //**WORKING**//
	  //**Used Generic Level Client Model**//
	  req.kind = mesh_generic_state_level;
	  req.level.level = 100;

	  LOG_INFO(" req.level.level %d",  req.level.level);


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
			  MESH_GENERIC_LEVEL_CLIENT_MODEL_ID,
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
  uint16 result;
  LOG_INFO("Establishing LPN");
  // Do not initialize LPN if lpn is currently active
  // or any GATT connection is opened
  if (lpn_active || num_connections)
  {
    return;
  }

  // Initialize LPN functionality.
  result = gecko_cmd_mesh_lpn_init()->result;
  if (result)
  {
	LOG_INFO("LPN init failed (0x%x)", result);
    return;
  }

  lpn_active = 1;
  LOG_INFO("LPN initialized");
#if ECEN5823_INCLUDE_DISPLAY_SUPPORT
		displayPrintf(DISPLAY_ROW_CONNECTION,"LPN Init done");
#endif

  // Configure the lpn with following parameters:
  // - Minimum friend queue length = 2
  // - Poll timeout = 5 seconds
  result = gecko_cmd_mesh_lpn_configure(2, 5 * 1000)->result;
  if (result)
  {
    LOG_INFO("LPN conf failed (0x%x)", result);
    return;
  }

  LOG_INFO("trying to find friend...");
  result = gecko_cmd_mesh_lpn_establish_friendship(0)->result;

  if (result != 0)
  {
	  LOG_INFO("ret.code %x", result);
  }
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

  result = gecko_cmd_hardware_set_soft_timer(0, // cancel friend finding timer
                                             TIMER_ID_FRIEND_FIND,
                                             1)->result;

  // Terminate friendship if exist
  result = gecko_cmd_mesh_lpn_terminate_friendship()->result;
  if (result)
  {
    LOG_INFO("Friendship termination failed (0x%x)", result);
  }
  // turn off lpn feature
  result = gecko_cmd_mesh_lpn_deinit()->result;
  if (result)
  {
    LOG_INFO("LPN deinit failed (0x%x)", result);
  }
  lpn_active = 0;
  LOG_INFO("LPN deinitialized");
#if ECEN5823_INCLUDE_DISPLAY_SUPPORT
		displayPrintf(DISPLAY_ROW_CONNECTION,"LPN De-Init done");
#endif
}


#if DEVICE_IS_ONOFF_PUBLISHER
#else

static struct reg *reg = NULL;
static size_t regs = 0;


static struct reg *find_reg(uint16_t model_id,
                            uint16_t elem_index)
{
  size_t r;
  for (r = 0; r < regs; r++) {
    if (reg[r].model_id == model_id && reg[r].elem_index == elem_index) {
      return &reg[r];
    }
  }
  return NULL;
}


/***************************************************************************//**
 * Sourced from SI labs Mesh Light Example (lighbulb.c)
 * Initialization of the models supported by this node.
 * This function registers callbacks for each of the supported models.
 ******************************************************************************/
static void init_models(void)
{
  mesh_lib_generic_server_register_handler(MESH_GENERIC_ON_OFF_SERVER_MODEL_ID,
                                           0,
										   client_request_cb,
										   state_changed_cb);

}



/***************************************************************************//**
 * Sourced from SI labs Mesh Light Example (lighbulb.c)
 * This function process the requests for the generic on/off model.
 *
 * @param[in] model_id       Server model ID.
 * @param[in] element_index  Server model element index.
 * @param[in] client_addr    Address of the client model which sent the message.
 * @param[in] server_addr    Address the message was sent to.
 * @param[in] appkey_index   The application key index used in encrypting the request.
 * @param[in] request        Pointer to the request structure.
 * @param[in] transition_ms  Requested transition time (in milliseconds).
 * @param[in] delay_ms       Delay time (in milliseconds).
 * @param[in] request_flags  Message flags. Bitmask of the following:
 *                           - Bit 0: Nonrelayed. If nonzero indicates
 *                                    a response to a nonrelayed request.
 *                           - Bit 1: Response required. If nonzero client
 *                                    expects a response from the server.
 ******************************************************************************/
static void client_request_cb(uint16_t model_id,
                          uint16_t element_index,
                          uint16_t client_addr,
                          uint16_t server_addr,
                          uint16_t appkey_index,
                          const struct mesh_generic_request *request,
                          uint32_t transition_ms,
                          uint16_t delay_ms,
                          uint8_t request_flags)
{

	LOG_INFO("model_id %d", model_id);

	LOG_INFO("element_index %d", element_index);

	LOG_INFO("request->on_off %d", request->on_off);
	if( request->on_off == MESH_GENERIC_ON_OFF_STATE_ON)
	{
		LOG_INFO("request->on_off %d", request->on_off);
#if ECEN5823_INCLUDE_DISPLAY_SUPPORT
  	  displayPrintf(DISPLAY_ROW_ACTION,"Button Pressed");
#endif
	}

	else if ( request->on_off == MESH_GENERIC_ON_OFF_STATE_OFF) //button released
	{
#if ECEN5823_INCLUDE_DISPLAY_SUPPORT
  	  displayPrintf(DISPLAY_ROW_ACTION,"Button Released");
#endif
	}
	onoff_update_and_publish(element_index, 0);
}



/***************************************************************************//**
 * Sourced from SI Labs Mesh Light Example
 * Update generic on/off state and publish model state to the network.
 *
 * @param[in] element_index  Server model element index.
 * @param[in] remaining_ms   The remaining time in milliseconds.
 *
 * @return Status of the update and publish operation.
 *         Returns bg_err_success (0) if succeed, non-zero otherwise.
 ******************************************************************************/
static errorcode_t onoff_update_and_publish(uint16_t element_index,
                                            uint32_t remaining_ms)
{
  errorcode_t e;

  e = onoff_update(element_index, remaining_ms);
  if (e == bg_err_success) {
    e = mesh_lib_generic_server_publish(MESH_GENERIC_ON_OFF_SERVER_MODEL_ID,
                                        element_index,
                                        mesh_generic_state_on_off);
    LOG_INFO("in onoff_update_and_publish: success %d",e);
  }

  return e;
}


/***************************************************************************//**
 * Sourced from SI Labs Mesh Light Example
 * Update generic on/off state.
 *
 * @param[in] element_index  Server model element index.
 * @param[in] remaining_ms   The remaining time in milliseconds.
 *
 * @return Status of the update operation.
 *         Returns bg_err_success (0) if succeed, non-zero otherwise.
 ******************************************************************************/
static errorcode_t onoff_update(uint16_t element_index, uint32_t remaining_ms)
{
  struct mesh_generic_state current, target;

  current.kind = mesh_generic_state_on_off;
  current.on_off.on = 0;

  target.kind = mesh_generic_state_on_off;
  target.on_off.on = 1;

  return mesh_lib_generic_server_update(MESH_GENERIC_ON_OFF_SERVER_MODEL_ID,
                                        element_index,
                                        &current,
                                        &target,
                                        remaining_ms);
}


/***************************************************************************//**
 * Sourced from SI Labs Light Change
 * This function is a handler for generic on/off change event.
 *
 * @param[in] model_id       Server model ID.
 * @param[in] element_index  Server model element index.
 * @param[in] current        Pointer to current state structure.
 * @param[in] target         Pointer to target state structure.
 * @param[in] remaining_ms   Time (in milliseconds) remaining before transition
 *                           from current state to target state is complete.
 ******************************************************************************/
static void state_changed_cb(uint16_t model_id,
                         uint16_t element_index,
                         const struct mesh_generic_state *current,
                         const struct mesh_generic_state *target,
                         uint32_t remaining_ms)
{
	LOG_INFO("In Onoff_change");
//  if (current->on_off.on != lightbulb_state.onoff_current) {
//    printf("on-off state changed %u to %u\r\n", lightbulb_state.onoff_current, current->on_off.on);
//
//    lightbulb_state.onoff_current = current->on_off.on;
//    lightbulb_state_changed();
//  } else {
//    printf("dummy onoff change - same state as before\r\n");
//  }
}

#endif
