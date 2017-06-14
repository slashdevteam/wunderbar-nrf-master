
/** @file   client_handling.c
 *  @brief  This driver contains functions for handling client events 
 *          and corresponding macros, constants,and global variables.
 *
 *  @author MikroElektronika
 *  @bug    No known bugs.
 */

#include "client_handling.h"
#include <string.h>
#include <stdbool.h>
#include "nrf.h"
#include "nrf_gpio.h"
#include "debug.h"
#include "ble_hci.h"
#include "spi_slave_config.h"
#include "onboard.h"

#define APPL_LOG                   debug_log      /**< Debug logger macro that will be used in this file to do logging of debug information over UART. */

#define IGNORE_LIST_NUM_OF_ENTRIES 10

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**@brief Extern variables. */

extern const ble_gap_scan_params_t * m_scan_param;   /**< Scan parameters requested for scanning and connection. */

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**@brief Static global variables. */

client_t               m_client[MAX_CLIENTS];                              /**< Client context information list. */
static ble_gap_addr_t  peer_addr_ignore_list[IGNORE_LIST_NUM_OF_ENTRIES];  /**< List of Bluetooth Low Energy Addresses which will be ignored. */
static uint16_t        ignore_list_index = 0;                              /**< Index of entry in IgnoreList which will be populated next. */
static bool            scan_start_flag = false;                            /**< State of scanning process (true if scanner running). */

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**@brief List of DeviceNames of sensors. */

const uint8_t  SENSORS_DEVICE_NAME[MAX_CLIENTS][BLE_DEVNAME_MAX_LEN + 1]  = LIST_OF_SENSOR_NAMES;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**@brief List of Characteristic UUID of sensors services. */

const uint16_t SENSOR_CHAR_UUIDS[NUMBER_OF_RELAYR_CHARACTERISTICS + 4] = LIST_OF_SENSOR_CHARS;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**@brief This function check if device name is in DEVICE_NAME list.
 *
 * @param  device_name           Device name to search.
 * @param  len                   Length of device name data.
 * @param  found_device_name     Pointer to entry in DEVICE_NAME list.
 *
 * @return true if input name is in list, other way false.
 *            
 */

bool validate_device_name(uint8_t * device_name, uint16_t len, const uint8_t ** found_device_name)
{
    uint8_t cnt;
    
    // Check if device is in run mode?
    if(onboard_get_mode() == ONBOARD_MODE_RUN)
    {
        for(cnt = 0; cnt < MAX_CLIENTS - 1; cnt++)
        {   
            if((memcmp(SENSORS_DEVICE_NAME[cnt],device_name, len) == 0) && (strlen((const char *)SENSORS_DEVICE_NAME[cnt]) == len))
            {
                *found_device_name = SENSORS_DEVICE_NAME[cnt];
                return true;
            }
        }
    }
		// Check if device is in onboard (config) mode?
    else if(onboard_get_mode() == ONBOARD_MODE_CONFIG)
    {
        if((memcmp(SENSORS_DEVICE_NAME[MAX_CLIENTS-1],device_name, len) == 0) && (strlen((const char *)SENSORS_DEVICE_NAME[MAX_CLIENTS-1]) == len))
        {
            *found_device_name = SENSORS_DEVICE_NAME[MAX_CLIENTS-1];
            return true;
        }
    }
    
    return false;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**@brief  This function check if "device_name sensor" is already connected.
 *
 * @param  device_name Device name to search.
 * @param  len         Length of device name data.
 *
 * @return true if sensor connected, other way false.
 *            
 */

client_t * is_device_connected(uint8_t * device_name, uint16_t len)
{
    uint8_t cnt;
  
    for(cnt = 0; cnt < MAX_CLIENTS; cnt++)
    {   
        if(0 == memcmp(m_client[cnt].device_name,device_name, len))
				{
				    if(m_client[cnt].state != STATE_IDLE)
					  {
						    return &m_client[cnt];
					  }
					  else
					  {
						    return NULL;
					  }
				}
    }
    return NULL;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**@brief This function add new entry to peer address ignore list.
 *
 * @param p_peer_addr Bluetooth Low Energy Addresses to be added
 *            
 * @return Void.
 */

void ignore_list_add(ble_gap_addr_t * p_peer_addr)
{   
    memcpy((uint8_t *)&peer_addr_ignore_list[ignore_list_index], (uint8_t *)p_peer_addr, sizeof(ble_gap_addr_t));
    ignore_list_index = (ignore_list_index + 1) % IGNORE_LIST_NUM_OF_ENTRIES;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**@brief This function go through ignore list searching input Bluetooth Low Energy Addresses.
 *
 * @param p_peer_addr Bluetooth Low Energy Addresses to be searched.
 *
 * @return    true if address found, other way false.           
 */

bool ignore_list_search(ble_gap_addr_t * p_peer_addr)
{
    uint16_t cnt;
  
    for(cnt = 0; cnt < ignore_list_index; cnt++)
    {
        if(memcmp((uint8_t *)&peer_addr_ignore_list[cnt], (uint8_t *)p_peer_addr, sizeof(ble_gap_addr_t)) == 0)
        {
            return true;
        }
    }
    return false;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**@brief This function check if there is client which is in discovery, identifying or notification_enable state.
 *
 * @return    Connection Handle of found client.
 */

uint16_t search_for_client_configuring(void)
{
    uint16_t cnt;
    for(cnt = 0; cnt < MAX_CLIENTS; cnt++)
    {   
        if(m_client[cnt].state < STATE_RUNNING) 
        {
            break;
        }
    }
    return cnt;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**@brief This function checks if there is client in ERROR state. In such case disconnect request for that client sends.
 *
 * @return Void.
 */

void search_for_client_error(void)
{
	  uint32_t err_code;
    uint16_t cnt;
    
    for(cnt = 0; cnt < MAX_CLIENTS; cnt++)
    {   
        if(m_client[cnt].state == STATE_ERROR)
        {
            err_code = sd_ble_gap_disconnect(m_client[cnt].srv_db.conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if(err_code == NRF_SUCCESS)
            {
                m_client[cnt].state = STATE_DISCONNECTING;
            }
            if(err_code > NRF_ERROR_BUSY)
            {
                m_client[cnt].state = STATE_IDLE;
            }
            
            return;
        }
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**@breif Function called to start scanning.
 *
 * @return Void.
 */ 

void scan_start(void)
{
    uint32_t err_code;
    if(scan_start_flag == false)
    {
        err_code = sd_ble_gap_scan_start(m_scan_param);
        APP_ERROR_CHECK(err_code);  
        scan_start_flag = true;
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**@breif Function called to stop scanning.
 *
 * @return Void.
 */ 

void scan_stop(void)
{
    uint32_t err_code;
    if(scan_start_flag == true)
    {
        err_code = sd_ble_gap_scan_stop();
        APP_ERROR_CHECK(err_code);  
        scan_start_flag = false;
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**@brief Function for finding client context information based on connection handle.
 *
 * @param conn_handle  Connection handle.
 *
 * @return client context information or NULL upon failure.
 */

static client_t * find_client_by_conn_handle(uint16_t conn_handle)
{
    uint32_t i;

    for (i = 0; i < MAX_CLIENTS; i++)
    {
        if (m_client[i].srv_db.conn_handle == conn_handle)
        {
            return &m_client[i];
        }
    }

    return NULL;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**@brief Function for finding client context information based on device name.
 *
 * @param device_name           Device name to search.
 * @param len                   Length of device name data.
 *
 * @return client context information or NULL upon failure.
 */

client_t * find_client_by_dev_name(const uint8_t * device_name, uint8_t len)
{
    uint8_t cnt;
  
    for(cnt = 0; cnt < MAX_CLIENTS; cnt++)
    {
        if(
            (m_client[cnt].state != STATE_IDLE) && 
            ( (device_name == m_client[cnt].device_name) || (memcmp(m_client[cnt].device_name,device_name, len) == 0))
          )
        {
            return &m_client[cnt];
        }
    }
    return NULL;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**@brief Function returns number of clients which is out of IDLE state.
 *
 * @return Number of "active" clients.
 */

uint8_t get_active_client_number(void)
{
    uint8_t cnt, active_num = 0;
  
    for(cnt = 0; cnt < MAX_CLIENTS; cnt++)
    {   
        if(m_client[cnt].state != STATE_IDLE)
        {
            active_num++;
        }
    }
    return active_num;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**@brief Function for finding characteristic based on characteristic short uuid and client context information.
 *
 * @param char_uuid  Characteristic short uuid.
 * @param p_client   Client context information short uuid.
 *
 * @return Address of found characterisitc.
 */

ble_db_discovery_char_t * find_char_by_uuid(uint16_t char_uuid, client_t * p_client)
{
  uint8_t cnt_srv, cnt_chr;
  ble_db_discovery_srv_t * service;
  
  for(cnt_srv = 0; cnt_srv < 3; cnt_srv++)
  {
      service = &p_client->srv_db.services[cnt_srv];
      for(cnt_chr = 0; cnt_chr < service->char_count; cnt_chr++)
      {
          if(service->charateristics[cnt_chr].characteristic.uuid.uuid == char_uuid)
          {
              return &(service->charateristics[cnt_chr]);
          }
      }
  }
  return NULL;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**@brief Function for finding characteristic based on characteristic handle value and client context information.
 *
 * @param char_uuid  Characteristic handle value.
 * @param p_client   Client context information short uuid.
 *
 * @return Address of found characterisitc.
 */

ble_db_discovery_char_t * find_char_by_handle_value(uint16_t handle_value, client_t * p_client)
{
  uint8_t cnt_srv, cnt_chr;
  ble_db_discovery_srv_t * service;
  
  for(cnt_srv = 0; cnt_srv < 3; cnt_srv++)
  {
      service = &p_client->srv_db.services[cnt_srv];
      for(cnt_chr = 0; cnt_chr < service->char_count; cnt_chr++)
      {
          if(service->charateristics[cnt_chr].characteristic.handle_value == handle_value)
          {
              return &(service->charateristics[cnt_chr]);
          }
      }
  }
  return NULL;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**@brief Function for start service discovery.
 *
 * @param p_client Client context information.
 *
 * @return Error code.
 */

static uint32_t service_discover(client_t * p_client)
{
    uint32_t   err_code;

    err_code = ble_db_discovery_start(&(p_client->srv_db), p_client->srv_db.conn_handle);
    return err_code;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**@brief Function for handling enabling notifications. Function found "next" characteristic with notification properties, and write to proper cccd.
 *
 * @param p_client Client context information.
 *
 * @return true on successful enabling, false if there is no more characteristics to "be enabled".
 */

static bool notif_enable(client_t * p_client)
{   
    uint8_t                  cnt_srv, cnt_chr;
    uint8_t                  buf[BLE_CCCD_VALUE_LEN];
    uint32_t                 err_code;
    ble_gattc_write_params_t write_params;
    ble_db_discovery_srv_t * service;
   
    // Search next characteristic with notification properties.
    for(cnt_srv = p_client->srv_index; cnt_srv < 3; cnt_srv++)
    {
        service = &p_client->srv_db.services[cnt_srv];
        for(cnt_chr = p_client->char_index; cnt_chr < service->char_count; cnt_chr++)
        {
            if(service->charateristics[cnt_chr].characteristic.char_props.notify == 1)
            {
                buf[0] = BLE_GATT_HVX_NOTIFICATION;
                buf[1] = 0;
              
                write_params.write_op = BLE_GATT_OP_WRITE_REQ;
                write_params.handle   = service->charateristics[cnt_chr].cccd_handle;
                write_params.offset   = 0;
                write_params.len      = sizeof(buf);
                write_params.p_value  = buf;
                APPL_LOG("[CL]: Request Notification Enable for %02x Characteristic\r\n", service->charateristics[cnt_chr].characteristic.uuid.uuid);
                err_code = sd_ble_gattc_write(p_client->srv_db.conn_handle, &write_params);
                APP_ERROR_CHECK(err_code);
                
                p_client->state = STATE_NOTIF_ENABLE;
                p_client->srv_index  = cnt_srv;
                p_client->char_index = cnt_chr + 1;
                return true;
            }
        }
        p_client->char_index = 0;
    }
    // There is no more characteristics to "be enabled".
    return false;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**@brief Function for writing to characteristic value.
 *
 * @param p_client Client context information.
 * @param uuid     Short UUID of characteristic.
 * @param data     Data that will be written.
 * @param len      Length of data.
 * @return    true in case of succeed write procedure, false in other case.
 */

bool write_characteristic_value(client_t * p_client, uint16_t uuid, uint8_t * data, uint16_t len)
{
    uint32_t                 err_code;
    ble_db_discovery_char_t * char_to_write;
    ble_gattc_write_params_t write_params;

    if(p_client->state != STATE_RUNNING) 
    {
        return false;
    }
    
    char_to_write = find_char_by_uuid(uuid, p_client);
    
    if(
       (char_to_write == NULL) ||
       (char_to_write->characteristic.char_props.write == 0)
      ) 
    {
        return false;
    }
    
    write_params.write_op = BLE_GATT_OP_WRITE_REQ;
    
    write_params.handle   = char_to_write->characteristic.handle_value;
    write_params.offset   = 0;
    write_params.len      = len;
    write_params.p_value  = data;

    err_code = sd_ble_gattc_write(p_client->srv_db.conn_handle, &write_params);
    APP_ERROR_CHECK(err_code);
    
    p_client->state = STATE_WAIT_WRITE_RSP;
    
    return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**@brief Function for initiate GATT Read of characteristic value
 *
 * @param p_client Client context information.
 * @param uuid     Short UUID of characteristic 
 * @return    true in case of succeed read request procedure, false in other case.
 */

bool read_characteristic_value(client_t * p_client, uint16_t uuid)
{
    uint32_t                 err_code;
    ble_db_discovery_char_t * char_to_read;

    if(p_client->state != STATE_RUNNING) 
    {
        return false;
    }
		
    APPL_LOG("[CL]: Initiate Read of %02x Characteristic Vlue\r\n", uuid);
    
    char_to_read = find_char_by_uuid(uuid, p_client);
    if(
       (char_to_read == NULL) ||
       (char_to_read->characteristic.char_props.read == 0)
      ) 
    {
        return false;
    }
    
    err_code = sd_ble_gattc_read(p_client->srv_db.conn_handle, char_to_read->characteristic.handle_value, 0);
    APP_ERROR_CHECK(err_code);
    
    p_client->state = STATE_WAIT_READ_RSP;
    return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**@brief Function for handling Relayr Service discovery events
 *
 * @param p_evt event from the DB discovery.
 *
 * @return Void.
 */

static void service_relayr_dsc_evt_handler(ble_db_discovery_evt_t * p_evt)
{
    uint32_t   err_code;
    client_t * p_client;
    ble_db_discovery_char_t * char_to_read = NULL;

    // Find the client using the connection handle.
    p_client = find_client_by_conn_handle(p_evt->conn_handle);
    p_client->state = STATE_ERROR;
   
    switch(p_evt->evt_type) 
    {
      
      case BLE_DB_DISCOVERY_COMPLETE:
      {
          APPL_LOG("[CL]: Discovery Relayr Complete\r\n");
          
          // If discoverred device is not "WunderbarApp" config device.
				  if(sensor_get_name_index(p_client->device_name) != DATA_ID_DEV_CFG_APP)
          { 
              char_to_read = find_char_by_uuid(CHARACTERISTIC_SENSOR_ID_UUID, p_client);
              if(char_to_read != NULL)
              {
                  err_code = sd_ble_gattc_read(p_client->srv_db.conn_handle, char_to_read->characteristic.handle_value, 0);
                  if(err_code == NRF_SUCCESS)
                  {
                      p_client->state = STATE_DEVICE_IDENTIFYING;
                  }
                  break;
              }
          }
          else
          {
              APPL_LOG("[CL]: Go to running state\r\n");  
              p_client->state = STATE_RUNNING;
          }
          
          break;
      }
      
      case BLE_DB_DISCOVERY_ERROR:
      {
          APPL_LOG("[CL]: Discovery Error\r\n");  
          break;
      }
       
      case BLE_DB_DISCOVERY_SRV_NOT_FOUND:
      {
          APPL_LOG("[CL]: Relayr Not Found\r\n");
          break;        
      }
      
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**@brief Function for handling Device Information Service discovery events
 *
 * @param p_evt event from the DB discovery .
 *
 * @return Void.
 */

static void service_deviceinf_dsc_evt_handler(ble_db_discovery_evt_t * p_evt)
{
    client_t * p_client;
    
    // Find the client using the connection handle.
    p_client = find_client_by_conn_handle(p_evt->conn_handle);
    
    switch(p_evt->evt_type) 
    {
      
        case BLE_DB_DISCOVERY_COMPLETE:
        {
            APPL_LOG("[CL]: Discovery Device Information Complete\r\n");  
            break;
        }
        
        case BLE_DB_DISCOVERY_ERROR:
        {
            APPL_LOG("[CL]: Discovery Error\r\n");  
            p_client->state = STATE_ERROR;  
            break;
        }
         
        case BLE_DB_DISCOVERY_SRV_NOT_FOUND:
        {
            if(sensor_get_name_index(p_client->device_name) != DATA_ID_DEV_CFG_APP)
            {
                APPL_LOG("[CL]: Discovery Device Information Not Found\r\n");
                p_client->state = STATE_ERROR;  
            }
            break;        
        }
      
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**@brief Function for handling Battery Service discovery events.
 *
 * @param p_evt event from the DB discovery .
 *
 * @return Void.
 */

static void service_batterylev_dsc_evt_handler(ble_db_discovery_evt_t * p_evt)
{
    // Find the client using the connection handle.
    client_t * p_client;

    p_client = find_client_by_conn_handle(p_evt->conn_handle);
    
    switch(p_evt->evt_type) 
    {
        case BLE_DB_DISCOVERY_COMPLETE:
        {
            APPL_LOG("[CL]: Discovery Battery Complete\r\n"); 
            break;
        }
        
        case BLE_DB_DISCOVERY_ERROR:
        {
            APPL_LOG("[CL]: Discovery Error\r\n");  
            p_client->state = STATE_ERROR;  
            break;
        }
         
        case BLE_DB_DISCOVERY_SRV_NOT_FOUND:
        {
            if(sensor_get_name_index(p_client->device_name) != DATA_ID_DEV_CFG_APP)
            {
                APPL_LOG("[CL]: Discovery Battery Not Found\r\n");
                p_client->state = STATE_ERROR;  
            }
            break;        
        }
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**@brief Function for handling write response received events.
 *
 * @param p_ble_evt Event to handle.
 *
 * @return Void.
 */

static void on_evt_write_rsp(ble_evt_t * p_ble_evt, client_t * p_client)
{
    if(p_client == NULL) 
    {
        return;
    }
    
    ble_gattc_evt_write_rsp_t * write_rsp = &p_ble_evt->evt.gattc_evt.params.write_rsp; 
    
    switch(p_client->state) {
        
			  // Setting client to the running state.
        case STATE_NOTIF_ENABLE:        
        {
            if (write_rsp->handle !=
                p_client->srv_db.services[p_client->srv_index].charateristics[p_client->char_index - 1].cccd_handle)
            {
                // Got response from unexpected handle.
                APPL_LOG("[CL]: Got response from unexpected handle\r\n");
                p_client->state = STATE_ERROR;
            }
            else
            {
                APPL_LOG("[CL]: Complete Notification Enable for %02x Characteristic\r\n", 
                          p_client->srv_db.services[p_client->srv_index].charateristics[p_client->char_index - 1].characteristic.uuid.uuid);
              
                // Search for more characteristics with notification properties.
                if(notif_enable(p_client) == false)
                {
                    data_id_t data_id;
                  
                    data_id = (data_id_t)sensor_get_name_index(p_client->device_name);
                    spi_create_tx_packet(data_id, FIELD_ID_SENSOR_STATUS, OPERATION_WRITE, p_client->id, sizeof(sensorID_t));
                    spi_lock_tx_packet(data_id);
                              
                    // All characterisitics with notification properties are enabled.
                    APPL_LOG("[CL]: Go to running state\r\n");
                  
                    p_client->state = STATE_RUNNING;
                  
                    if (get_active_client_number() < DEVICE_MANAGER_MAX_CONNECTIONS)
                    {
                        scan_start();
                    }
                }
            }
            break;
        }
        
				// Send OK write response through SPI.
        case STATE_WAIT_WRITE_RSP:
        { 
            spi_create_tx_packet(DATA_ID_RESPONSE_OK, 0xFF, 0xFF, NULL, 0);
            p_client->state = STATE_RUNNING;
            break;
        }     
        
        default:
        {
            break;
        }   
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**@brief Function for handling read response received events.
 *
 * @param p_ble_evt Event to handle.
 * @param p_client  Client context information.
 *
 * @return Void.
 */

static void on_evt_read_rsp(ble_evt_t * p_ble_evt, client_t * p_client)
{
    uint8_t cnt;
  
    ble_gattc_evt_read_rsp_t * read_rsp = &p_ble_evt->evt.gattc_evt.params.read_rsp;  
    
    APPL_LOG("[CL]: Receive response of handle: %x -> ", p_ble_evt->evt.gattc_evt.params.read_rsp.handle);
    for(cnt = 0; cnt < read_rsp->len; cnt++)
    {
        APPL_LOG("%02x", read_rsp->data[cnt]);
    }
    
    APPL_LOG("\r\n");

    if(p_client == NULL)
    {
        return;
    }
    
    switch(p_client->state) 
    { 
      
        case STATE_DEVICE_IDENTIFYING:
        {   
          
            memcpy((uint8_t *)p_client->id, (uint8_t *)&read_rsp->data, read_rsp->len);
            p_client->char_index = 0;
            p_client->srv_index  = 0;
            notif_enable(p_client);
            break;
        } 
      
        case STATE_WAIT_READ_RSP:
        {   
            data_id_t data_id;
            uint8_t char_id;
            ble_db_discovery_char_t * characterisitc;
            
            p_client->state = STATE_RUNNING;
          
            data_id = (data_id_t)sensor_get_name_index(p_client->device_name);
            characterisitc = find_char_by_handle_value(read_rsp->handle, p_client); 
            char_id = sensor_get_char_index(characterisitc->characteristic.uuid.uuid);
            
            if(
                (onboard_get_state() == ONBOARD_STATE_IDLE) &&
                (data_id != DATA_ID_DEV_CFG_APP) 
              )
            {
                spi_create_tx_packet(data_id, char_id, OPERATION_WRITE, read_rsp->data, read_rsp->len);
            }
            else if(
                     (onboard_get_state() != ONBOARD_STATE_IDLE) &&
                     (data_id == DATA_ID_DEV_CFG_APP) 
                   )
            {  
                onboard_parse_data(char_id, read_rsp->data, read_rsp->len);
            }  
            break;
        }
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**@brief Function for handling notification received events.
 *
 * @param p_ble_evt Event to handle.
 * @param p_client  Client context information.
 *
 * @return Void.
 */

static void on_evt_hvx(ble_evt_t * p_ble_evt, client_t * p_client)
{
    uint8_t cnt;
    if (
			  (p_client != NULL) && 
			  ((p_client->state == STATE_RUNNING)||(p_client->state == STATE_WAIT_WRITE_RSP)||(p_client->state == STATE_WAIT_READ_RSP))
		   )
    {   
        data_id_t            data_id;
        ble_gattc_evt_hvx_t *     hvx;
        uint8_t    char_id;
        ble_db_discovery_char_t * characterisitc;
      
        hvx = &p_ble_evt->evt.gattc_evt.params.hvx;
      
        data_id = (data_id_t)sensor_get_name_index(p_client->device_name);
      
        characterisitc = find_char_by_handle_value(hvx->handle, p_client);  
        char_id = sensor_get_char_index(characterisitc->characteristic.uuid.uuid);
        
        spi_create_tx_packet(data_id, char_id, OPERATION_WRITE, hvx->data, hvx->len);  
      
        APPL_LOG("[CL]: Notification-> ConHandle:  %d; Handle:  0x%X; Device Name: %s;  Value: 0x", 
                p_client->srv_db.conn_handle, hvx->handle, p_client->device_name);
        
        for(cnt = 0; cnt < hvx->len; cnt++)
        {
            APPL_LOG("%02X", hvx->data[cnt]);
        }
        APPL_LOG("\r\n");
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**@brief Function for handling timeout events.
 *
 * @param p_ble_evt Event to handle.
 * @param p_client  Client context information.
 *
 * @return Void.
 */

static void on_evt_timeout(ble_evt_t * p_ble_evt, client_t * p_client)
{
    APPL_LOG("[CL]: Event Timeout Occur\r\n");  
    APP_ERROR_CHECK_BOOL(p_ble_evt->evt.gattc_evt.params.timeout.src == BLE_GATT_TIMEOUT_SRC_PROTOCOL);

    if (p_client != NULL)
    {
        p_client->state = STATE_ERROR;
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**@brief Function for handling device manager events.
 *
 * @param p_handle      Device Handle.
 * @param p_event       Event type.
 * @param event_result  Status of API procedure.
 *
 * @return Void.
 */

api_result_t client_handling_dm_event_handler(const dm_handle_t    * p_handle,
                                              const dm_event_t     * p_event,
                                              const api_result_t     event_result)
{
    client_t * p_client = &m_client[p_handle->connection_id];

    switch (p_event->event_id)
    {
         case DM_EVT_LINK_SECURED:
             // Attempt configuring CCCD now that bonding is established.
             if (event_result == NRF_SUCCESS)
             {
                 notif_enable(p_client);
             }
             break;
         default:
             break;
    }

    return NRF_SUCCESS;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**@brief Function for handling ble events.
 *
 * @param p_ble_evt Event to handle.
 *
 * @return Void.
 */

void client_handling_ble_evt_handler(ble_evt_t * p_ble_evt)
{
    client_t * p_client = NULL;
  
    p_client = find_client_by_conn_handle(p_ble_evt->evt.gattc_evt.conn_handle);
    
    if(p_client  == NULL)
    {
        return;
    }

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GATTC_EVT_WRITE_RSP:
            if ((p_ble_evt->evt.gattc_evt.gatt_status == BLE_GATT_STATUS_ATTERR_INSUF_AUTHENTICATION) ||
                (p_ble_evt->evt.gattc_evt.gatt_status == BLE_GATT_STATUS_ATTERR_INSUF_ENCRYPTION))
            {
                uint32_t err_code = dm_security_setup_req(&p_client->handle);
                APP_ERROR_CHECK(err_code);

            }
            on_evt_write_rsp(p_ble_evt, p_client);
            break;
            
        case BLE_GATTC_EVT_READ_RSP:
            if ((p_ble_evt->evt.gattc_evt.gatt_status == BLE_GATT_STATUS_ATTERR_INSUF_AUTHENTICATION) ||
                (p_ble_evt->evt.gattc_evt.gatt_status == BLE_GATT_STATUS_ATTERR_INSUF_ENCRYPTION))
            {
                uint32_t err_code = dm_security_setup_req(&p_client->handle);
                APP_ERROR_CHECK(err_code);

            }
            on_evt_read_rsp(p_ble_evt, p_client);
            break;

        case BLE_GATTC_EVT_HVX:
            on_evt_hvx(p_ble_evt, p_client);
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            on_evt_timeout(p_ble_evt, p_client);
            break;

        default:
            break;
    }


    if (p_client != NULL)
    {
        ble_db_discovery_on_ble_evt(&(p_client->srv_db), p_ble_evt);
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**@brief Database discovery module initialization.
 *
 * @return Void.
 */

static void db_discovery_init(void)
{
    ble_db_discovery_init_t db_discovery_init_obj;

    uint32_t err_code = ble_db_discovery_init(&db_discovery_init_obj);
    APP_ERROR_CHECK(err_code);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**@brief Function for initializing the client handling.
 *
 * @return Void.
 */

void client_handling_init(void)
{
    uint32_t err_code;
    uint32_t i;

    nrf_gpio_range_cfg_output(8, 15);

    for (i = 0; i < MAX_CLIENTS; i++)
    {
        m_client[i].state  = STATE_IDLE;
    }

    db_discovery_init();

    // Register with discovery module for the discovery of the service.
    ble_uuid_t uuid;
  
    uuid.type = BLE_UUID_TYPE_BLE;
    uuid.uuid = SHORT_SERVICE_RELAYR_UUID;

    err_code = ble_db_discovery_register(&uuid, service_relayr_dsc_evt_handler);
                                                
    uuid.type = BLE_UUID_TYPE_BLE;
    uuid.uuid = BLE_UUID_BATTERY_SERVICE;

    err_code = ble_db_discovery_register(&uuid, service_deviceinf_dsc_evt_handler);
    
    uuid.type = BLE_UUID_TYPE_BLE;
    uuid.uuid = BLE_UUID_DEVICE_INFORMATION_SERVICE;

    err_code = ble_db_discovery_register(&uuid, service_batterylev_dsc_evt_handler);

    APP_ERROR_CHECK(err_code);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**@brief Function for creating a new client.
 *
 * @return Void.
 */

uint32_t client_handling_create(const dm_handle_t * p_handle, uint16_t conn_handle, current_conn_device_t * current_conn_device)
{
    uint32_t err_code;
  
    m_client[p_handle->connection_id].srv_db.conn_handle = conn_handle;
    m_client[p_handle->connection_id].handle             = (*p_handle);
    m_client[p_handle->connection_id].device_name        = current_conn_device->device_name;
    memcpy( (uint8_t *)&m_client[p_handle->connection_id].peer_addr, (uint8_t *)&current_conn_device->peer_addr, sizeof(ble_gap_addr_t));
    err_code = service_discover(&m_client[p_handle->connection_id]);
  
    if(err_code == NRF_SUCCESS)
    {
        m_client[p_handle->connection_id].state = STATE_SERVICE_DISC;
    }
    else 
    {
        m_client[p_handle->connection_id].state = STATE_ERROR;
    }
  
    return err_code;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**@brief Function for freeing up a client by setting its state to idle.
 *
 * @return Void.
 */

uint32_t client_handling_destroy(const dm_handle_t * p_handle)
{
    uint32_t   err_code = NRF_SUCCESS;
    client_t * p_client = &m_client[p_handle->connection_id];

  
    if (p_client->state != STATE_IDLE)
    {
        data_id_t data_id;                    

        data_id = (data_id_t)sensor_get_name_index(p_client->device_name);
        memset((uint8_t *)p_client->id, 0, 8);
        spi_create_tx_packet(data_id, FIELD_ID_SENSOR_STATUS, OPERATION_READ, NULL, 0);
      
        p_client->state = STATE_IDLE;
    }
    else
    {
        err_code = NRF_ERROR_INVALID_STATE;
    }
    
    return err_code;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
