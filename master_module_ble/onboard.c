
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

#define SEC_PARAM_BOND                   1                                              /**< Perform bonding. */
#define SEC_PARAM_OOB                    0                                              /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE           7                                              /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE           16                                             /**< Maximum encryption key size. */

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

const ble_gap_sec_params_t * sec_params;

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

const ble_gap_conn_params_t * m_connection_param;

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

extern const uint8_t    SENSORS_DEVICE_NAME[MAX_CLIENTS][BLE_DEVNAME_MAX_LEN + 1];
extern passkey_t        sensors_passkey[MAX_CLIENTS];

onboard_mode_t  onboard_mode  = ONBOARD_MODE_IDLE;
onboard_state_t onboard_state = ONBOARD_STATE_IDLE;

bool onboard_store_passkeys = false;

static const uint16_t service_uuid_list_config_mode[3] = {SHORT_SERVICE_CONFIG_UUID, BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_BATTERY_SERVICE};
static const uint16_t service_uuid_list_run_mode[3]    = {SHORT_SERVICE_RELAYR_UUID, BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_BATTERY_SERVICE};

/** @brief  Set onboarding mode.
 *
 *  @param  new_mode  Mode to be set.
 *
 *  @return  Void.
 */

void onboard_set_mode(onboard_mode_t new_mode)
{
    APPL_LOG("[OB]: Set mode %d\r\n", new_mode);
    onboard_mode = new_mode;
}

/** @brief  Get onboarding mode.
 *
 *  @return  Current onboarding mode.
 */

onboard_mode_t onboard_get_mode(void)
{
    return onboard_mode;
}

/** @brief  Set onboarding state.
 *
 *  @param  new_mode  State to be set.
 *
 *  @return  Void.
 */

void onboard_set_state(onboard_state_t new_state)
{
    APPL_LOG("[OB]: Set state %d\r\n", new_state);
    onboard_state = new_state;
}

void onboard_set_sec_params(onboard_mode_t onboard_mode)
{
    if (ONBOARD_MODE_CONFIG == onboard_mode) {
        sec_params = &sec_params_config_mode;
        m_connection_param = &m_connection_param_config_mode;
        m_scan_param = &m_scan_param_config_mode;
    } else {
        sec_params = &sec_params_run_mode;
        m_connection_param = &m_connection_param_run_mode;
        m_scan_param = &m_scan_param_run_mode;
    }
}

/** @brief  Get onboarding state.
 *
 *  @return  Current onboarding state.
 */

onboard_state_t onboard_get_state(void)
{
    return onboard_state;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** @brief  Function which is called on successful store data in persistent memory.
 *
 *  @return  Void.
 */
void onboard_on_store_complete(void)
{

    switch(onboard_state)
    {

        case ONBOARD_STATE_STORING_HTU_PASS:
        {
            if(onboard_store_passkey_from_wifi(DATA_ID_DEV_GYRO) == false)
            {
                onboard_set_state(ONBOARD_STATE_ERROR);
            }
            break;
        }

        case ONBOARD_STATE_STORING_GYRO_PASS:
        {
            if(onboard_store_passkey_from_wifi(DATA_ID_DEV_LIGHT) == false)
            {
                onboard_set_state(ONBOARD_STATE_ERROR);
            }
            break;
        }

        case ONBOARD_STATE_STORING_LIGHT_PASS:
        {
            if(onboard_store_passkey_from_wifi(DATA_ID_DEV_SOUND) == false)
            {
                onboard_set_state(ONBOARD_STATE_ERROR);
            }
            break;
        }

        case ONBOARD_STATE_STORING_SOUND_PASS:
        {
            if(onboard_store_passkey_from_wifi(DATA_ID_DEV_BRIDGE) == false)
            {
                onboard_set_state(ONBOARD_STATE_ERROR);
            }
            break;
        }

        case ONBOARD_STATE_STORING_BRIDGE_PASS:
        {
            if(onboard_store_passkey_from_wifi(DATA_ID_DEV_IR) == false)
            {
                onboard_set_state(ONBOARD_STATE_ERROR);
            }
            break;
        }


        case ONBOARD_STATE_STORING_IR_PASS:
        {
            onboard_state++;
            break;
        }

        default:
        {
            spi_create_tx_packet(DATA_ID_DEV_CFG_APP, FIELD_ID_CONFIG_ACK, OPERATION_WRITE, NULL, 0);
        }
    }
}

/** @brief  Function saves locally passkey received from kinetis mcu.
 *
 *  @return  Void.
 */
void onboard_save_passkey_from_wifi(uint8_t passkey_index, uint8_t * data)
{
    memcpy((uint8_t*)&sensors_passkey[passkey_index], data, PASSKEY_SIZE);
}


/** @brief  Function sends request to copy passkey from local memory to NVRAM.
 *
 *  @return  false in case that error is occurred, otherwise true.
 */
bool onboard_store_passkey_from_wifi(uint8_t passkey_index)
{
    onboard_state++;

    return pstorage_driver_request_store((uint8_t*)(&sensors_passkey[passkey_index]));
}

/** @brief  Function handles various states of onboarding process.
 *
 *  @return  Void.
 */

void onboard_state_handle(void)
{
    if(onboard_state == ONBOARD_STATE_IDLE)
    {
        return;
    }

    switch(onboard_state)
    {
        case ONBOARD_STATE_START:
        {
            APPL_LOG("[OB]: Start config, store passkeys: %d.\r\n", onboard_store_passkeys);
            if (onboard_store_passkeys)
            {
                if (onboard_store_passkey_from_wifi(DATA_ID_DEV_HTU) == false)
                {
                    onboard_set_state(ONBOARD_STATE_ERROR);
                }
            }
            else
            {
                onboard_set_state(ONBOARD_STATE_COMPLETE);
            }
            break;
        }

        case ONBOARD_STATE_ERROR:
        {
            APPL_LOG("[OB]: Config ERROR.\r\n");
            spi_create_tx_packet(DATA_ID_DEV_CFG_APP, FIELD_ID_CONFIG_ERROR, NOT_USED, NULL, 0);
            onboard_set_state(ONBOARD_STATE_IDLE);
            break;
        }

        case ONBOARD_STATE_COMPLETE:
        {
            APPL_LOG("[OB]: Config complete.\r\n");
            spi_create_tx_packet(DATA_ID_DEV_CFG_APP, FIELD_ID_CONFIG_COMPLETE, NOT_USED, NULL, 0);
            onboard_set_state(ONBOARD_STATE_IDLE);
            break;
        }

        default:{}
    }
}

 const uint16_t* onboard_get_service_list(void) {
     const uint16_t* serv_list = service_uuid_list_config_mode;

    if (ONBOARD_MODE_RUN == onboard_get_mode()) {
        serv_list = service_uuid_list_run_mode;
    }
    return serv_list;
}

void onboard_set_store_passkeys()
{
    onboard_store_passkeys = true;
}

