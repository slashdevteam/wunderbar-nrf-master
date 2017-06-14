
/** @file   main.c
 *  @brief  This file contains the source code for firmware of master board of Wunderbar Board.
 *
 *  @author MikroElektronika
 *  @bug    No known bugs.
 */
 
/* -- Includes -- */

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "ble.h"
#include "client_handling.h"
#include "softdevice_handler.h"
#include "nrf6310.h"
#include "pstorage_driver.h"
#include "device_manager.h"
#include "debug.h"
#include "spi_slave_config.h"
#include "onboard.h"

#define APPL_LOG                         debug_log                                      /**< Debug logger macro that will be used in this file to do logging of debug information over UART. */

#define SEC_PARAM_BOND                   1                                              /**< Perform bonding. */
#define SEC_PARAM_MITM                   1                                              /**< Man In The Middle protection required. */
#define SEC_PARAM_OOB                    0                                              /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE           7                                              /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE           16                                             /**< Maximum encryption key size. */

#define MAX_PEER_COUNT                   DEVICE_MANAGER_MAX_CONNECTIONS                 /**< Maximum number of peer's application intends to manage. */
#define UUID16_SIZE                      2                                              /**< Size of 16 bit UUID */

const uint8_t CENTRAL_BLE_FIRMWARE_REV[20] = "1.0.0";

/**@breif Macro to unpack 16bit unsigned UUID from octet stream. */
#define UUID16_EXTRACT(DST,SRC)                                                                  \
        do                                                                                       \
        {                                                                                        \
            (*(DST)) = (SRC)[1];                                                                 \
            (*(DST)) <<= 8;                                                                      \
            (*(DST)) |= (SRC)[0];                                                                \
        } while(0)

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**@brief Variable length data encapsulation in terms of length and pointer to data */
        
typedef struct
{
    uint8_t  * p_data;          /**< Pointer to data. */
    uint16_t   data_len;        /**< Length of data. */
}
data_t;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** @brief Security requirements for this application. */

static const ble_gap_sec_params_t  sec_params_run_mode = 
{
    SEC_PARAM_BOND,               // bond
    1,                            // mitm
    BLE_GAP_IO_CAPS_KEYBOARD_ONLY,// io_caps
    SEC_PARAM_OOB,                // oob
    SEC_PARAM_MIN_KEY_SIZE,       // min_key_size
    SEC_PARAM_MAX_KEY_SIZE        // max_key_size 
};

static const ble_gap_sec_params_t  sec_params_config_mode = 
{
    SEC_PARAM_BOND,               // bond
    0,                            // mitm
    BLE_GAP_IO_CAPS_NONE,         // io_caps
    SEC_PARAM_OOB,                // oob
    SEC_PARAM_MIN_KEY_SIZE,       // min_key_size
    SEC_PARAM_MAX_KEY_SIZE        // max_key_size 
};

static const ble_gap_sec_params_t * sec_params;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** @brief Connection parameters requested for connection. */

static const ble_gap_conn_params_t m_connection_param_run_mode =
{
    (uint16_t)MIN_CONNECTION_INTERVAL,   // Minimum connection
    (uint16_t)MAX_CONNECTION_INTERVAL,   // Maximum connection
    (uint16_t)SLAVE_LATENCY,             // Slave latency
    (uint16_t)SUPERVISION_TIMEOUT        // Supervision time-out
};

static const ble_gap_conn_params_t m_connection_param_config_mode =
{
    (uint16_t)MSEC_TO_UNITS(50, UNIT_1_25_MS), // Minimum connection
    (uint16_t)MSEC_TO_UNITS(50, UNIT_1_25_MS), // Maximum connection
    (uint16_t)SLAVE_LATENCY,             // Slave latency
    (uint16_t)SUPERVISION_TIMEOUT        // Supervision time-out
};

static const ble_gap_conn_params_t * m_connection_param;
	
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**@brief Scan parameters requested for scanning and connection. */

static const ble_gap_scan_params_t m_scan_param_run_mode =
{
     0,                       // Active scanning set.
     0,                       // Selective scanning not set.
     NULL,                    // White-list not set.
     (uint16_t)SCAN_INTERVAL, // Scan interval.
     (uint16_t)SCAN_WINDOW,   // Scan window.
     0                        // Never stop scanning unless explicit asked to.
};

static const ble_gap_scan_params_t m_scan_param_config_mode =
{
     0,                       // Active scanning set.
     0,                       // Selective scanning not set.
     NULL,                    // White-list not set.
     (uint16_t)MSEC_TO_UNITS(50, UNIT_0_625_MS), // Scan interval.
     (uint16_t)MSEC_TO_UNITS(30, UNIT_0_625_MS), // Scan window.
     0                        // Never stop scanning unless explicit asked to.
};

const ble_gap_scan_params_t * m_scan_param;
	
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**@brief  Default values for sensors passkeys. These values are used if corresponding block of persistent storage is empty. */
const uint8_t  DEFAULT_SENSOR_PASSKEY[8]   = {0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x00, 0x00};

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static dm_application_instance_t m_dm_app_id;              /**< Application identifier. */
static current_conn_device_t     current_conn_device;

passkey_t  sensors_passkey[MAX_CLIENTS] __attribute__((aligned(4)));

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**@brief Function for error handling, which is called when an error has occurred.
 *
 * @param error_code      Error code supplied to the handler.
 * @param line_num        Line number where the handler is called.
 * @param p_file_name     Pointer to the file name.
 *
 * @return Void.
 */
 
void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
{
    APPL_LOG("[AP]: ASSERT: %s, %d, error 0x%08x\r\n", p_file_name, line_num, error_code);

    // This call can be used for debug purposes during development of an application.
    // @note CAUTION: Activating this code will write the stack to flash on an error.
    //                This function should NOT be used in a final product.
    //                It is intended STRICTLY for development/debugging purposes.
    //                The flash write will happen EVEN if the radio is active, thus interrupting
    //                any communication.
    //                Use with care. Un-comment the line below to use.
    // ble_debug_assert_handler(error_code, line_num, p_file_name);
    
    // On assert, the system can only recover with a reset.
    NVIC_SystemReset();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**@brief Function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param line_num        Line number of the failing ASSERT call.
 * @param p_file_name     File name of the failing ASSERT call.
 *
 * @return Void.
 */

void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(0xDEADBEEF, line_num, p_file_name);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**@brief Callback handling device manager events.
 *
 * @details This function is called to notify the application of device manager events.
 *
 * @param   p_handle      Device Manager Handle. For link related events, this parameter
 *                            identifies the peer.
 * @param   p_event       Pointer to the device manager event.
 * @param   event_status  Status of the event.
 *
 * @retval  Status of API procedure.
 */

static api_result_t device_manager_event_handler(const dm_handle_t    * p_handle,
                                                 const dm_event_t     * p_event,
                                                 const api_result_t     event_result)
{
    uint32_t   err_code;
	
    switch(p_event->event_id)
    {
        case DM_EVT_CONNECTION:
        { 
            APPL_LOG("[AP]: [0x%02X] >> DM_EVT_CONNECTION\r\n", p_handle->connection_id);
            ble_gap_addr_t * p_peer_addr;
            p_peer_addr = &p_event->event_param.p_gap_param->params.connected.peer_addr;
            APPL_LOG("[AP]: [%02X %02X %02X %02X %02X %02X]: Connection Established -> %s\r\n",
                            p_peer_addr->addr[0], p_peer_addr->addr[1], p_peer_addr->addr[2],
                            p_peer_addr->addr[3], p_peer_addr->addr[4], p_peer_addr->addr[5], current_conn_device.device_name);
            
            if(memcmp((uint8_t *)&current_conn_device.peer_addr, (uint8_t *)p_peer_addr, sizeof(ble_gap_addr_t)) == 0)
            {
                APPL_LOG("[AP]: [CI 0x%02X]: Requesting GAP Authenticate\r\n", p_handle->connection_id);
                
							  dm_handle_t handle = (*p_handle);
							  err_code = dm_security_setup_req(&handle);
                APP_ERROR_CHECK(err_code);
							
            }
            else
            {
                APPL_LOG("[AP]: Wrong Peer Address\r\n");
                sd_ble_gap_disconnect(p_handle->connection_id, 0x13);
            }
            
              
            APPL_LOG("[AP]: [0x%02X] << DM_EVT_CONNECTION\r\n", p_handle->connection_id);
            APPL_LOG("\r\n");
            break;
        }
        
        case DM_EVT_DISCONNECTION:
        {
            uint16_t conf_client_connection_id;
            APPL_LOG("[AP]: [0x%02X] >> DM_EVT_DISCONNECTION\r\n", p_handle->connection_id);
            
            // Try to destroy client.
            err_code = client_handling_destroy(p_handle);
            
            conf_client_connection_id = search_for_client_configuring();
            if(
                (conf_client_connection_id == MAX_CLIENTS) ||
                (conf_client_connection_id == p_handle->connection_id)
              )
            {
                scan_start();
            }
            
            APPL_LOG("[AP]: [0x%02X] << DM_EVT_DISCONNECTION\r\n", p_handle->connection_id);
            break;
        }
        
        case DM_EVT_SECURITY_SETUP_COMPLETE:
        {
            APPL_LOG("[AP]: [0x%02X] >> DM_EVT_SECURITY_SETUP_COMPLETE, result 0x%08X\r\n", p_handle->connection_id, event_result);
            
            if(event_result == NRF_SUCCESS)
            {
                APPL_LOG("[AP]: [CI 0x%02X]: Requesting GATT client create\r\n", p_handle->connection_id);
                err_code = client_handling_create(p_handle, p_event->event_param.p_gap_param->conn_handle, &current_conn_device);
                if(err_code != NRF_SUCCESS)
                {
                    sd_ble_gap_disconnect(p_handle->connection_id, 0x13);
                }
            }
            else
            {
							  if(onboard_get_mode() != ONBOARD_MODE_CONFIG)
								{
										ignore_list_add(&current_conn_device.peer_addr);
								}
                sd_ble_gap_disconnect(p_handle->connection_id, 0x13);
            }
            
            APPL_LOG("[AP]: [0x%02X] << DM_EVT_SECURITY_SETUP_COMPLETE\r\n", p_handle->connection_id);
            
            break;
        }
        
        case DM_EVT_SECURITY_SETUP:
        {
            APPL_LOG("[AP]: [0x%02X] >> DM_EVT_SECURITY_SETUP\r\n", p_handle->connection_id);
            APPL_LOG("[AP]: [0x%02X] << DM_EVT_SECURITY_SETUP\r\n", p_handle->connection_id);
            break;
        }
        
        case DM_EVT_LINK_SECURED:
        {
            APPL_LOG("[AP]: [0x%02X] >> DM_LINK_SECURED_IND, result 0x%08X\r\n", p_handle->connection_id, event_result);
            APPL_LOG("[AP]: [0x%02X] << DM_LINK_SECURED_IND\r\n", p_handle->connection_id);
					  
					  if(current_conn_device.bonded_flag == true)
						{
								err_code = client_handling_create(p_handle, p_event->event_param.p_gap_param->conn_handle, &current_conn_device);
								if(err_code != NRF_SUCCESS)
								{
										sd_ble_gap_disconnect(p_handle->connection_id, 0x13);
								}
						}
					
            break;
        }
        
        case DM_EVT_SECURITY_SETUP_REFRESH:
        {
            APPL_LOG("[AP]: [0x%02X] >> DM_EVT_SECURITY_SETUP_REFRESH, result 0x%08X\r\n", p_handle->connection_id, event_result);
            APPL_LOG("[AP]: [0x%02X] << DM_EVT_SECURITY_SETUP_REFRESH\r\n", p_handle->connection_id);
            break;
        }
        
        case DM_EVT_DEVICE_CONTEXT_LOADED:
        {
            APPL_LOG("[AP]: [0x%02X] >> DM_EVT_LINK_SECURED\r\n", p_handle->connection_id);
            APP_ERROR_CHECK(event_result);
            APPL_LOG("[AP]: [0x%02X] << DM_EVT_DEVICE_CONTEXT_LOADED\r\n", p_handle->connection_id);
					  current_conn_device.bonded_flag = true;
            break;
        }
        
        case DM_EVT_DEVICE_CONTEXT_STORED:
        {
            APPL_LOG("[AP]: [0x%02X] >> DM_EVT_DEVICE_CONTEXT_STORED\r\n", p_handle->connection_id);
            APP_ERROR_CHECK(event_result);
            APPL_LOG("[AP]: [0x%02X] << DM_EVT_DEVICE_CONTEXT_STORED\r\n", p_handle->connection_id);
            break;
        }
        
        case DM_EVT_DEVICE_CONTEXT_DELETED:
        {
            APPL_LOG("[AP]: [0x%02X] >> DM_EVT_DEVICE_CONTEXT_DELETED\r\n", p_handle->connection_id);
            APP_ERROR_CHECK(event_result);
            APPL_LOG("[AP]: [0x%02X] << DM_EVT_DEVICE_CONTEXT_DELETED\r\n", p_handle->connection_id);
            break;
        }
        
        case DM_EVT_ERROR:
        {
            APPL_LOG("[AP]: [0x%02X] >> DM_EVT_ERROR\r\n", p_handle->connection_id);
            APP_ERROR_CHECK(event_result);
            APPL_LOG("[AP]: [0x%02X] << DM_EVT_ERROR\r\n", p_handle->connection_id);
            break;
        }
        
        case DM_EVT_APPL_CONTEXT_DELETED:
        {
            APPL_LOG("[AP]: [0x%02X] >> DM_EVT_APPL_CONTEXT_DELETED\r\n", p_handle->connection_id);
            APP_ERROR_CHECK(event_result);
            APPL_LOG("[AP]: [0x%02X] << DM_EVT_APPL_CONTEXT_DELETED\r\n", p_handle->connection_id);
            break;
        }

        default:
            break;
    }

    return NRF_SUCCESS;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Parses advertisement data, providing length and location of the field in case
 *        matching data is found.
 *
 * @param  Type of data to be looked for in advertisement data.
 * @param  Advertisement report length and pointer to report.
 * @param  If data type requested is found in the data report, type data length and
 *             pointer to data will be populated here.
 *
 * @return NRF_SUCCESS if the data type is found in the report.
 * @return NRF_ERROR_NOT_FOUND if the data type could not be found.
 */

static uint32_t adv_report_parse(uint8_t type, data_t * p_advdata, data_t * p_typedata)
{
    uint32_t index = 0;
    uint8_t * p_data;

    p_data = p_advdata->p_data;

    while (index < p_advdata->data_len)
    {
        uint8_t field_length = p_data[index];
        uint8_t field_type   = p_data[index+1];

        if (field_type == type)
        {
            p_typedata->p_data   = &p_data[index+2];
            p_typedata->data_len = field_length-1;
            return NRF_SUCCESS;
        }
        index += field_length+1;
    }
    return NRF_ERROR_NOT_FOUND;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Function for handling the Application's BLE Stack events.
 *
 * @param  p_ble_evt   Bluetooth stack event.
 *
 * @return Void.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t         err_code;
    static uint16_t  service_uuid_list[3] = {SHORT_SERVICE_RELAYR_UUID, BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_BATTERY_SERVICE};

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_ADV_REPORT:
        {
            data_t adv_data;
            data_t type_data;
            
            if(onboard_get_mode() == ONBOARD_MODE_IDLE)
            {
                return;
            }
          
            ble_gap_addr_t * peer_addr = &p_ble_evt->evt.gap_evt.params.adv_report.peer_addr;
          
            if(ignore_list_search(peer_addr) == true)
            {
                APPL_LOG("[AP]: Device is in ignore list\r\n");
                return;
            }
            
            // Initialize advertisement report for parsing.
            adv_data.p_data = p_ble_evt->evt.gap_evt.params.adv_report.data;
            adv_data.data_len = p_ble_evt->evt.gap_evt.params.adv_report.dlen;
            
            err_code = adv_report_parse(BLE_GAP_AD_TYPE_16BIT_SERVICE_UUID_COMPLETE, &adv_data, &type_data);
            
            // Verify if list of services matches target. 
            if( (err_code == NRF_SUCCESS) && (memcmp((uint8_t *)&service_uuid_list, type_data.p_data, type_data.data_len) == 0))
            {
                err_code = adv_report_parse(BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME, &adv_data, &type_data);
              
                // Verify if complete name matches target.
                if (err_code == NRF_SUCCESS)   
                {
                    const uint8_t * found_device_name;
                  
                    if(validate_device_name(type_data.p_data, type_data.data_len, &found_device_name) == false)
                    {
                        APPL_LOG("[AP]: Invalid device name. Adding to ignore list\r\n");
                        ignore_list_add(&p_ble_evt->evt.gap_evt.params.adv_report.peer_addr);
                    }
                    else if(find_client_by_dev_name(type_data.p_data, type_data.data_len) != NULL)
                    {
                        APPL_LOG("[AP]: Device with this name is already connected.\r\n");
                    }
                    else
                    {
                        APPL_LOG("\r\n[AP]: Found device %s\r\n\r\n", found_device_name);
                        scan_stop();
											  
												err_code = sd_ble_gap_connect(&p_ble_evt->evt.gap_evt.params.adv_report.peer_addr, m_scan_param, m_connection_param);
												if (err_code == NRF_SUCCESS)
												{
														memcpy((uint8_t *)&current_conn_device.peer_addr, (uint8_t *)peer_addr, sizeof(ble_gap_addr_t));      
													  current_conn_device.bonded_flag = false;
														current_conn_device.device_name = found_device_name;
												}
												else
												{
														APPL_LOG("[AP]: Connection Request Failed, reason %d\r\n", err_code);
												}
												
                    }
                }
            }
            break;
        }
        
        case BLE_GAP_EVT_TIMEOUT:
        {  
          
            APPL_LOG("[AP]: BLE_GAP_EVT_TIMEOUT.\r\n");
          
            if(p_ble_evt->evt.gap_evt.params.timeout.src == BLE_GAP_TIMEOUT_SRC_SCAN)
            {
                APPL_LOG("[AP]: Scan Timedout.\r\n");
            }
            else if (p_ble_evt->evt.gap_evt.params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
            {
                APPL_LOG("[AP]: Connection Request Timedout.\r\n");
            }
            break;
        }
        
        
        case BLE_GAP_EVT_AUTH_KEY_REQUEST:
        {
            uint8_t passkey_index;
            APPL_LOG("[AP]: Authentication Key Request Received\r\n");
            passkey_index = sensor_get_name_index(current_conn_device.device_name);
            err_code = sd_ble_gap_auth_key_reply(p_ble_evt->evt.gap_evt.conn_handle, BLE_GAP_AUTH_KEY_TYPE_PASSKEY, sensors_passkey[passkey_index]);
            APP_ERROR_CHECK(err_code);
            APPL_LOG("[AP]: Authentication Key Response Send -> %s\r\n", sensors_passkey[passkey_index]);
            break;
        }
        
        case BLE_GAP_EVT_CONN_SEC_UPDATE:
        {
            break;
        }
        
        default:
            break;
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the scheduler in the main loop after a BLE stack event has
 *  been received.
 *
 * @param   p_ble_evt     Bluetooth stack event.
 *
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    dm_ble_evt_handler(p_ble_evt);
    client_handling_ble_evt_handler(p_ble_evt);
    on_ble_evt(p_ble_evt);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**@brief Function for dispatching a system event to interested modules.
 *
 * @details This function is called from the System event interrupt handler after a system
 *          event has been received.
 *
 * @param[in]   sys_evt   System stack event.
 *
 * @return Void.
 */
static void sys_evt_dispatch(uint32_t sys_evt)
{
    pstorage_sys_event_handler(sys_evt);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 *
 * @return Void.
 */

static void ble_stack_init(void)
{
    uint32_t err_code;
	  ble_gap_conn_sec_mode_t sec_mode;

    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_XTAL_20_PPM, false);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for System events.
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);
	
	
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&sec_mode);

	  sd_ble_gap_device_name_set(&sec_mode,(uint8_t const *)DEVICE_NAME_MASTER_BLE,strlen(DEVICE_NAME_MASTER_BLE));
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**@brief Function for initializing the Device Manager.
 *
 * @details Device manager is initialized here.
 *
 * @param   sec_params  Pointer to security params structure.
 *
 * @return Void.
 */

static void device_manager_init(const ble_gap_sec_params_t * sec_params)
{
    dm_application_param_t param;    
    dm_init_param_t        init_param;

    uint32_t err_code;

    // Clear all bonded devices if user requests to.
    init_param.clear_persistent_data = true;

    err_code = dm_init(&init_param);
    APP_ERROR_CHECK(err_code);

    memset(&param.sec_param, 0, sizeof (ble_gap_sec_params_t));

    // Event handler to be registered with the module.
    param.evt_handler            = device_manager_event_handler;

    // Service or protocol context for device manager to load, store and apply on behalf of application.
    // Here set to client as application is a GATT client.
    param.service_type           = DM_PROTOCOL_CNTXT_GATT_CLI_ID;

    // Secuirty parameters to be used for security procedures.
		memcpy((uint8_t*)&param.sec_param, (uint8_t*)sec_params, sizeof(ble_gap_sec_params_t));
		
		param.sec_param.kdist_periph.enc   = 1;
    param.sec_param.kdist_periph.id    = 1;

    err_code = dm_register(&m_dm_app_id,&param);
    APP_ERROR_CHECK(err_code);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**@brief   Inits global with value stored in persistent_storage, or with default_value (if corresponding block in persistent_storage is empty).
 *
 * @param   global          Global which will be initialized.
 * @param   default_value   If corresponding block in persistent_storage is empty, initialize with this param.
 * @param   default_value   Size of global param.
 *
 * @return  true if operation is successful, otherwise false.
 */

bool init_global(uint8_t * global, uint8_t * default_value, uint16_t size) 
{
    uint32_t load_status;
   
    // Set pstorage block id.  	
    if (!pstorage_driver_register_block(global, size)) 
    {
        return false;
    }
		
		// Load data from corresponding block.
    load_status = pstorage_driver_load(global);
    if((load_status == PS_LOAD_STATUS_FAIL)||(load_status == PS_LOAD_STATUS_NOT_FOUND)) 
    {
        return false;
    }
		// If there is no data use default value.
    else if(load_status == PS_LOAD_STATUS_EMPTY) 
    { 
        memcpy(global, default_value, size);
    }
    return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**@brief  This function initializes and configures pstorage. 
 *         Also, it registers characteristic values to corresponding blocks in persistent memory.
 *
 * @param  None.
 *
 * @return true if operation is successful, otherwise false.
 */

bool pstorage_driver_init() 
{
    uint32_t err_code;
    
    err_code = pstorage_init();
    APP_ERROR_CHECK(err_code);
    
    // Configure pstorage driver.			
    if(!pstorage_driver_cfg(0x20)) 
    {
        return false;
    }
    
    // Read passkeys from p_storage.
		
    if(!init_global((uint8_t*)&sensors_passkey[DATA_ID_DEV_HTU], (uint8_t*)&DEFAULT_SENSOR_PASSKEY, sizeof(passkey_t))) 
    {
        return false;
    }
    
    if(!init_global((uint8_t*)&sensors_passkey[DATA_ID_DEV_GYRO], (uint8_t*)&DEFAULT_SENSOR_PASSKEY, sizeof(passkey_t))) 
    {
        return false;
    }
    
    if(!init_global((uint8_t*)&sensors_passkey[DATA_ID_DEV_LIGHT], (uint8_t*)&DEFAULT_SENSOR_PASSKEY, sizeof(passkey_t))) 
    {
        return false;
    }
    
    if(!init_global((uint8_t*)&sensors_passkey[DATA_ID_DEV_SOUND], (uint8_t*)&DEFAULT_SENSOR_PASSKEY, sizeof(passkey_t))) 
    {
        return false;
    }
    
    if(!init_global((uint8_t*)&sensors_passkey[DATA_ID_DEV_BRIDGE], (uint8_t*)&DEFAULT_SENSOR_PASSKEY, sizeof(passkey_t))) 
    {
        return false;
    }
    
    if(!init_global((uint8_t*)&sensors_passkey[DATA_ID_DEV_IR], (uint8_t*)&DEFAULT_SENSOR_PASSKEY, sizeof(passkey_t))) 
    {
        return false;
    }
    
    return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**@brief Function for the Power manager.
 *
 * @return Void.
 *
 */

static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**@brief Function for application main entry.
 */

int main(void)
{
    // Initialization of various modules.
    debug_init();
    ble_stack_init();
    client_handling_init();
    pstorage_driver_init();
    spi_slave_app_init();
    
	  // Wait to go out of IDLE mode.
	  while(onboard_get_mode() == ONBOARD_MODE_IDLE)
		{
				power_manage();
		}
		
		// Set params depending on the mode.
		if(onboard_get_mode() == ONBOARD_MODE_RUN)
		{
				sec_params = &sec_params_run_mode;
			  m_connection_param = &m_connection_param_run_mode;
			  m_scan_param = &m_scan_param_run_mode;
		}
		else
		{
				sec_params = &sec_params_config_mode;
			  m_connection_param = &m_connection_param_config_mode;
			  m_scan_param = &m_scan_param_config_mode;
		}
		
	  device_manager_init(sec_params);
		
    // Start scanning for devices.
    APPL_LOG("\r\n[AP]: Start Scan\r\n\r\n");
    scan_start();
  
    for (;;)
    {
        power_manage();
        onboard_state_handle();
        pstorage_driver_run();
        spi_check_tx_ready();
        search_for_client_error();  
    }
}
