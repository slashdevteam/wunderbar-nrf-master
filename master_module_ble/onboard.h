
/** @file   onboard.c
 *  @brief  This driver contains functions for working with onboarding process 
 *          and corresponding macros, constants,and global variables.
 *
 *  @author MikroElektronika
 *  @bug    No known bugs.
 */
 
#ifndef ONBOARD_H__
#define ONBOARD_H__

#include "wunderbar_common.h"

#define ONBOARD_SENSOR_PASS_LEN  6
#define ONBOARD_WIFI_SSID_LEN    19
#define ONBOARD_WIFI_PASS_LEN    19
#define ONBOARD_MASTER_ID_LEN    16
#define ONBOARD_MASTER_SEC_LEN   12
#define ONBOARD_MASTER_URL_LEN   19

/**@brief Onboard Characteristic UUIDs. */
#define ONBOARD_CHAR_HTU_GYRO_LIGHT_PASS      0x2010
#define ONBOARD_CHAR_SOUND_BRIDGE_IR_PASS     0x2011
#define ONBOARD_CHAR_WIFI_SSID                0x2012
#define ONBOARD_CHAR_WIFI_PASS                0x2013
#define ONBOARD_CHAR_MASTER_MODULE_ID         0x2014
#define ONBOARD_CHAR_MASTER_MODULE_SEC        0x2015
#define ONBOARD_CHAR_MASTER_MODULE_URL        0x2016

/**< List of used characteristics UUID. */
#define LIST_OF_ONBOARD_CHARS  {                                   \
                                ONBOARD_CHAR_HTU_GYRO_LIGHT_PASS,  \
                                ONBOARD_CHAR_SOUND_BRIDGE_IR_PASS, \
                                ONBOARD_CHAR_WIFI_SSID,            \
                                ONBOARD_CHAR_WIFI_PASS,            \
                                ONBOARD_CHAR_MASTER_MODULE_ID,     \
                                ONBOARD_CHAR_MASTER_MODULE_SEC,    \
	                              ONBOARD_CHAR_MASTER_MODULE_URL     \
                               }

/**< Number of characteristics in Relayr Service. */
#define NUMBER_OF_ONBOARD_CHARACTERISTICS                      7

/**< List of characteristics UUID index. */
typedef enum
{
    ONBOARD_CHAR_INDEX_HTU_GYRO_LIGHT_PASS   = 0,
    ONBOARD_CHAR_INDEX_SOUND_BRIDGE_IR_PASS  = 1,
    ONBOARD_CHAR_INDEX_WIFI_SSID             = 2,
    ONBOARD_CHAR_INDEX_WIFI_PASS             = 3,
    ONBOARD_CHAR_INDEX_MASTER_MODULE_ID      = 4,
    ONBOARD_CHAR_INDEX_MASTER_MODULE_SEC     = 5,
	  ONBOARD_CHAR_INDEX_MASTER_MODULE_URL     = 6
}
onboard_characteristics_t;

/**< Onboarding mode. */
typedef enum
{
    ONBOARD_MODE_IDLE   = 0,
    ONBOARD_MODE_CONFIG = 1,
    ONBOARD_MODE_RUN    = 2
}
onboard_mode_t;

/**< Onboarding state. */
typedef enum
{
    ONBOARD_STATE_IDLE                       = 0,
    ONBOARD_STATE_START                      = 1,
  
    ONBOARD_STATE_WAIT_HTU_GYRO_LIGHT_PASS   = 2,
    ONBOARD_STATE_STORING_HTU_PASS           = 3,
    ONBOARD_STATE_STORING_GYRO_PASS          = 4,
	  ONBOARD_STATE_STORING_LIGHT_PASS         = 5,
  
    ONBOARD_STATE_WAIT_SOUND_BRIDGE_IR_PASS  = 6,
    ONBOARD_STATE_STORING_SOUND_PASS         = 7,
	  ONBOARD_STATE_STORING_BRIDGE_PASS        = 8,
    ONBOARD_STATE_STORING_IR_PASS            = 9,
  
  
    ONBOARD_STATE_WAIT_WIFI_SSID             = 10,
    ONBOARD_STATE_SENDING_WIFI_SSID          = 11,
  
    ONBOARD_STATE_WAIT_WIFI_PASS             = 12,
    ONBOARD_STATE_SENDING_WIFI_PASS          = 13, 
  
    ONBOARD_STATE_WAIT_MASTER_MODULE_ID      = 14,
    ONBOARD_STATE_SENDING_MASTER_MODULE_ID   = 15, 
  
    ONBOARD_STATE_WAIT_MASTER_MODULE_SEC     = 16,
    ONBOARD_STATE_SENDING_MASTER_MODULE_SEC  = 17, 
		
		ONBOARD_STATE_WAIT_MASTER_MODULE_URL     = 18,
		ONBOARD_STATE_SENDING_MASTER_MODULE_URL  = 19, 
    
    ONBOARD_STATE_COMPLETE                   = 20,
    
    ONBOARD_STATE_ERROR                      = 30
}
onboard_state_t;

onboard_mode_t onboard_get_mode(void);
void onboard_set_mode(onboard_mode_t new_mode);
void onboard_set_state(onboard_state_t new_state);
void onboard_on_store_complete(void);
void onboard_on_send_complete(void);
void onboard_parse_data(uint8_t field_id, uint8_t * data, uint8_t len);
onboard_state_t onboard_get_state(void);
void onboard_state_handle(void);
bool onboard_store_passkey_from_ble(uint8_t passkey_index, uint8_t * data);
bool onboard_store_passkey_from_wifi(uint8_t passkey_index, uint8_t * data);

#endif // ONBOARD_H__

