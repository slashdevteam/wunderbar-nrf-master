
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
#define PASSKEY_SIZE ONBOARD_SENSOR_PASS_LEN

/**@brief  Default values for sensors passkeys. These values are used if corresponding block of persistent storage is empty. */
// static const uint8_t DEFAULT_SENSOR_PASSKEY[PASSKEY_SIZE+2]   = {0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x00, 0x00};
static const uint8_t DEFAULT_SENSOR_PASSKEY[PASSKEY_SIZE+2]   = {0x33, 0x34, 0x33, 0x34, 0x33, 0x34, 0x00, 0x00};
// const uint8_t  DEFAULT_SENSOR_PASSKEY[8]   = {0x31, 0x31, 0x31, 0x31, 0x31, 0x31, 0x31, 0x31};

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
    ONBOARD_STATE_IDLE                       = 0x0,
    ONBOARD_STATE_START                      = 0x1,
    
    ONBOARD_STATE_STORING_HTU_PASS           = 0x2,
    ONBOARD_STATE_STORING_GYRO_PASS          = 0x3,
    ONBOARD_STATE_STORING_LIGHT_PASS         = 0x4,
    ONBOARD_STATE_STORING_SOUND_PASS         = 0x5,
    ONBOARD_STATE_STORING_BRIDGE_PASS        = 0x6,
    ONBOARD_STATE_STORING_IR_PASS            = 0x7,
    
    ONBOARD_STATE_COMPLETE                   = 0x8,
    
    ONBOARD_STATE_ERROR                      = 0x9
}
onboard_state_t;

onboard_mode_t onboard_get_mode(void);
void onboard_set_mode(onboard_mode_t new_mode);
void onboard_set_state(onboard_state_t new_state);
void onboard_set_sec_params(onboard_mode_t onboard_mode);
void onboard_on_store_complete(void);
onboard_state_t onboard_get_state(void);
void onboard_state_handle(void);
void onboard_save_passkey_from_wifi(uint8_t passkey_index, uint8_t * data);
bool onboard_store_passkey_from_wifi(uint8_t passkey_index);
const uint16_t* onboard_get_service_list(void);
void onboard_set_store_passkeys();

#endif // ONBOARD_H__

