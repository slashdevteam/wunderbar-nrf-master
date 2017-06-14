
/** @file   onboard.c
 *  @brief  This driver contains functions for working with onboarding process 
 *          and corresponding macros, constants,and global variables.
 *
 *  @author MikroElektronika
 *  @bug    No known bugs.
 */

#include "onboard.h"
#include "client_handling.h"
#include "spi_slave_config.h"
#include "pstorage_driver.h"
#include "debug.h"

#define APPL_LOG        debug_log      /**< Debug logger macro that will be used in this file to do logging of debug information over UART. */

extern const uint8_t    SENSORS_DEVICE_NAME[NUMBER_OF_SENSORS][BLE_DEVNAME_MAX_LEN + 1];
extern passkey_t        sensors_passkey[MAX_CLIENTS];

const  uint16_t         ONBOARD_CHAR_UUIDS[NUMBER_OF_ONBOARD_CHARACTERISTICS] = LIST_OF_ONBOARD_CHARS;

onboard_mode_t  onboard_mode  = ONBOARD_MODE_IDLE;
onboard_state_t onboard_state = ONBOARD_STATE_IDLE;
static onboard_characteristics_t current_char;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** @brief  Set onboarding mode.
 *
 *  @param  new_mode  Mode to be set.
 *
 *  @return  Void.
 */
 
void onboard_set_mode(onboard_mode_t new_mode)
{
    onboard_mode = new_mode;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** @brief  Get onboarding mode.
 *
 *  @return  Current onboarding mode.
 */

onboard_mode_t onboard_get_mode(void)
{
    return onboard_mode;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** @brief  Set onboarding state.
 *
 *  @param  new_mode  State to be set.
 *
 *  @return  Void.
 */

void onboard_set_state(onboard_state_t new_state)
{
    onboard_state = new_state;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** @brief  Get onboarding state.
 *
 *  @return  Current onboarding state.
 */

onboard_state_t onboard_get_state(void)
{
    return onboard_state;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** @brief  Function which is called on successful send onboarding data to kinetis mcu.
 *
 *  @return Void.
 */

void onboard_on_send_complete(void)
{
    switch(onboard_state)
    {
        case ONBOARD_STATE_SENDING_WIFI_SSID:
        case ONBOARD_STATE_SENDING_WIFI_PASS:
        case ONBOARD_STATE_SENDING_MASTER_MODULE_ID:
				case ONBOARD_STATE_SENDING_MASTER_MODULE_SEC:	
        {
            client_t * p_client;  
            p_client = find_client_by_dev_name(SENSORS_DEVICE_NAME[DATA_ID_DEV_CFG_APP], 0);
            if(p_client != NULL)
            {
                read_characteristic_value(p_client, ONBOARD_CHAR_UUIDS[current_char++]);
            }
            
            onboard_state++;
            break;
        }
				
        case ONBOARD_STATE_SENDING_MASTER_MODULE_URL: 
        {
            onboard_state++;
            break;
        }
        
        default:{}
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** @brief  Function which is called on successful store data in persistent memory.
 *
 *  @return  Void.
 */

void onboard_on_store_complete(void)
{ 
    client_t * p_client;  
      
    switch(onboard_state)
    {
      
        case ONBOARD_STATE_STORING_HTU_PASS:
        {
            if(onboard_store_passkey_from_ble(DATA_ID_DEV_GYRO, NULL) == false)
            {
                onboard_set_state(ONBOARD_STATE_ERROR);
            }
            break;
        }
        
        case ONBOARD_STATE_STORING_GYRO_PASS:
        {
            if(onboard_store_passkey_from_ble(DATA_ID_DEV_LIGHT, NULL) == false)
            {
                onboard_set_state(ONBOARD_STATE_ERROR);
            }
            break;
        }
        
        case ONBOARD_STATE_STORING_SOUND_PASS:
        {
            if(onboard_store_passkey_from_ble(DATA_ID_DEV_BRIDGE, NULL) == false)
            {
                onboard_set_state(ONBOARD_STATE_ERROR);
            }
            break;
        }
				
				case ONBOARD_STATE_STORING_BRIDGE_PASS:
        {
            if(onboard_store_passkey_from_ble(DATA_ID_DEV_IR, NULL) == false)
            {
                onboard_set_state(ONBOARD_STATE_ERROR);
            }
            break;
        }
      
        case ONBOARD_STATE_STORING_LIGHT_PASS:
        case ONBOARD_STATE_STORING_IR_PASS: 
        {
            p_client = find_client_by_dev_name(SENSORS_DEVICE_NAME[DATA_ID_DEV_CFG_APP], 0);
            if(p_client != NULL)
            {
                read_characteristic_value(p_client, ONBOARD_CHAR_UUIDS[current_char++]);
            }
            onboard_state++;
            break;
        } 
        
        default:
        {
            spi_create_tx_packet(DATA_ID_DEV_CFG_APP, FIELD_ID_CONFIG_ACK, OPERATION_WRITE, NULL, 0);
        }
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** @brief  Function send request to storing passkey received from kinetis mcu.
 *
 *  @return  false in case that error is occurred, otherwise true.
 */

bool onboard_store_passkey_from_wifi(uint8_t passkey_index, uint8_t * data)
{
    memcpy((uint8_t*)&sensors_passkey[passkey_index], data, 6); 
    return pstorage_driver_request_store((uint8_t*)(&sensors_passkey[passkey_index]));
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** @brief  Function send request to storing passkey received through bluetooth interface.
 *
 *  @return  false in case that error is occurred, otherwise true.
 */

bool onboard_store_passkey_from_ble(uint8_t passkey_index, uint8_t * data)
{
    static uint8_t * current_pass;
	  static uint8_t   valid_flag;
    
    onboard_state++;
  
    if(data != NULL)
    {
        current_pass = data;
			  valid_flag = data[3*ONBOARD_SENSOR_PASS_LEN];
			
        if(valid_flag & 0x1)
        {
            memcpy((uint8_t*)&sensors_passkey[passkey_index], current_pass, 6); 
            return pstorage_driver_request_store((uint8_t*)(&sensors_passkey[passkey_index]));
        }
        else
        {
            onboard_on_store_complete();
            return true;
        }
    }
    else
    {
        current_pass += ONBOARD_SENSOR_PASS_LEN;
			  valid_flag >>= 1;
			
        if(valid_flag & 0x1)
        {
            memcpy((uint8_t*)&sensors_passkey[passkey_index], current_pass, 6); 
            return pstorage_driver_request_store((uint8_t*)(&sensors_passkey[passkey_index]));
        }
        else
        {
            onboard_on_store_complete();
            return true;
        }
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** @brief  Function parse onboarding data which is received through bluetooth interface.
 *
 *  @return  Void.
 */

void onboard_parse_data(uint8_t field_id, uint8_t * data, uint8_t len)
{
    switch(onboard_state)
    {
        case ONBOARD_STATE_WAIT_HTU_GYRO_LIGHT_PASS:
        {
            if(onboard_store_passkey_from_ble(DATA_ID_DEV_HTU, data) == false)
            {
                onboard_set_state(ONBOARD_STATE_ERROR);
            }
            break;
        }
        
        case ONBOARD_STATE_WAIT_SOUND_BRIDGE_IR_PASS:
        {
            if(onboard_store_passkey_from_ble(DATA_ID_DEV_SOUND, data) == false)
            {
                onboard_set_state(ONBOARD_STATE_ERROR);
            }
            break;
        }
        
        case ONBOARD_STATE_WAIT_WIFI_SSID:
        {
            onboard_state++;
            if(data[ONBOARD_WIFI_SSID_LEN] == true)
            {
                spi_create_tx_packet(DATA_ID_DEV_CFG_APP, FIELD_ID_CONFIG_WIFI_SSID, OPERATION_WRITE, data, len);
            }
            else
            {
                onboard_on_send_complete();
            }
            break;
        }
        
        case ONBOARD_STATE_WAIT_WIFI_PASS:
        {
            onboard_state++;
            if(data[ONBOARD_WIFI_PASS_LEN] == true)
            {
                spi_create_tx_packet(DATA_ID_DEV_CFG_APP, FIELD_ID_CONFIG_WIFI_PASS, OPERATION_WRITE, data, len);
            }
            else
            {
                onboard_on_send_complete();
            }
            break;
        }
        
        case ONBOARD_STATE_WAIT_MASTER_MODULE_ID:
        {
            onboard_state++;
            if(data[ONBOARD_MASTER_ID_LEN] == true)
            {
                spi_create_tx_packet(DATA_ID_DEV_CFG_APP, FIELD_ID_CONFIG_MASTER_MODULE_ID, OPERATION_WRITE, data, len);
            }
            else
            {
                onboard_on_send_complete();
            }
            break;
        }
        
        case ONBOARD_STATE_WAIT_MASTER_MODULE_SEC:
        {
            onboard_state++;
            if(data[ONBOARD_MASTER_SEC_LEN] == true)
            {
                spi_create_tx_packet(DATA_ID_DEV_CFG_APP, FIELD_ID_CONFIG_MASTER_MODULE_SEC, OPERATION_WRITE, data, ONBOARD_MASTER_SEC_LEN);
            }
            else
            {
                onboard_on_send_complete();
            }
            break;
        }
				
				case ONBOARD_STATE_WAIT_MASTER_MODULE_URL:
        {
            onboard_state++;
            if(data[ONBOARD_MASTER_URL_LEN] == true)
            {
                spi_create_tx_packet(DATA_ID_DEV_CFG_APP, FIELD_ID_CONFIG_MASTER_MODULE_URL, OPERATION_WRITE, data, ONBOARD_MASTER_URL_LEN);
            }
            else
            {
                onboard_on_send_complete();
            }
            break;
        }
        
        default:
        {
            return;
        }
    }
}
  
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** @brief  Function handles various states of onboarding process.
 *
 *  @return  Void.
 */

void onboard_state_handle(void)
{  
    client_t * p_client;  
    
    if(onboard_state == ONBOARD_STATE_IDLE)
    {
        return;
    }
  
    p_client = find_client_by_dev_name(SENSORS_DEVICE_NAME[DATA_ID_DEV_CFG_APP], 0);
    if( 
        (p_client == NULL) ||
        ((p_client->state != STATE_RUNNING) && (p_client->state != STATE_WAIT_READ_RSP))
      )
    {
        // Onboarding is started. Waiting device.
        if(onboard_get_state() == ONBOARD_STATE_START)
        {
            return;
        }
        // Config device is disconnected during onboarding process.
        else
        {
            onboard_set_state(ONBOARD_STATE_ERROR);
        }
        
    }
  
    switch(onboard_state)
    {
        case ONBOARD_STATE_IDLE:
        {
            break;
        }
        
        case ONBOARD_STATE_START:
        {
            APPL_LOG("[OB]: Start config.\r\n");  
            current_char = ONBOARD_CHAR_INDEX_HTU_GYRO_LIGHT_PASS;
            read_characteristic_value(p_client, ONBOARD_CHAR_UUIDS[current_char++]);
            onboard_set_state(ONBOARD_STATE_WAIT_HTU_GYRO_LIGHT_PASS);
            break;
        }
        
        case ONBOARD_STATE_ERROR:
        {
            APPL_LOG("[OB]: Config ERROR.\r\n");  
            spi_create_tx_packet(DATA_ID_DEV_CFG_APP, FIELD_ID_CONFIG_ERROR, OPERATION_WRITE, NULL, 0);
            onboard_set_state(ONBOARD_STATE_IDLE);
            break;
        }
        
        case ONBOARD_STATE_COMPLETE:
        {
            APPL_LOG("[OB]: Config complete.\r\n"); 
            spi_create_tx_packet(DATA_ID_DEV_CFG_APP, FIELD_ID_CONFIG_COMPLETE, OPERATION_WRITE, NULL, 0);
            onboard_set_state(ONBOARD_STATE_IDLE);
            break;
        }
        
        default:{}
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

