
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


extern const uint8_t    client_device_names[MAX_CLIENTS][BLE_DEVNAME_MAX_LEN + 1];
extern passkey_t        sensors_passkeys[MAX_CLIENTS];
extern sensorID_t       client_device_uuids[MAX_CLIENTS];
extern serivce_desc_t   discovery_services[MAX_DISCOVERY_SERVICES];
extern uint8_t          discovery_services_index;
extern uint8_t          client_char_uuids_index[MAX_CLIENTS];
extern char_desc_t      client_char_uuids[MAX_CLIENTS][NUMBER_OF_RELAYR_CHARACTERISTICS + 4];
// const  uint16_t         ONBOARD_CHAR_UUIDS[NUMBER_OF_ONBOARD_CHARACTERISTICS] = LIST_OF_ONBOARD_CHARS;

onboard_mode_t  onboard_mode  = ONBOARD_MODE_IDLE;
onboard_state_t onboard_state = ONBOARD_STATE_IDLE;
// static onboard_characteristics_t current_char;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** @brief Security requirements for this application. */

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

const ble_gap_conn_params_t* m_connection_param;

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

const ble_gap_scan_params_t* m_scan_param;

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

const ble_gap_sec_params_t* sec_params;

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
    APPL_LOG("[OB]: Set mode %d\r\n", new_mode);
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
    APPL_LOG("[OB]: Set state %d\r\n", new_state);
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

void onboard_set_run_security_params(void)
{
    sec_params = &sec_params_run_mode;
    m_connection_param = &m_connection_param_run_mode;
    m_scan_param = &m_scan_param_run_mode;
}

void onboard_set_discovery_security_params(void)
{
    sec_params = &sec_params_config_mode;
    m_connection_param = &m_connection_param_config_mode;
    m_scan_param = &m_scan_param_config_mode;
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
            // client_t * p_client;
            // p_client = find_client_by_dev_name(client_device_names[DATA_ID_DEV_CFG_APP], 0);
            // if(p_client != NULL)
            // {
            //     read_characteristic_value(p_client, ONBOARD_CHAR_UUIDS[current_char++]);
            // }

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
    // client_t * p_client;

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
            // p_client = find_client_by_dev_name(client_device_names[DATA_ID_DEV_CFG_APP], 0);
            // if(p_client != NULL)
            // {
            //     read_characteristic_value(p_client, ONBOARD_CHAR_UUIDS[current_char++]);
            // }
            onboard_state++;
            break;
        }

        default:
        {
            spi_create_tx_packet(DATA_ID_DEV_CENTRAL, FIELD_ID_CONFIG_ACK, OPERATION_WRITE, NULL, 0);
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
bool onboard_store_discovery_service(uint8_t* data)
{
    if(MAX_DISCOVERY_SERVICES > discovery_services_index)
    {
        memcpy((uint8_t*)&discovery_services[discovery_services_index], data, sizeof(serivce_desc_t));
        APPL_LOG("[OB]: discovery_services[%d] = %x %x %x\r\n",
                 discovery_services_index,
                 discovery_services[discovery_services_index].type,
                 discovery_services[discovery_services_index].uuid,
                 discovery_services[discovery_services_index].use_mode);
        discovery_services_index++;
        return true;
    }
    return false;
}

bool onboard_store_client_char_uuid(uint8_t client_index, uint8_t* data)
{
    uint8_t client_char_index = client_char_uuids_index[client_index];
    if(MAX_NUMBER_OF_CHARACTERISTICS > client_char_index)
    {
        memcpy((uint8_t*)&client_char_uuids[client_index][client_char_index], data, sizeof(char_desc_t));
        client_char_uuids_index[client_index]++;
        return true;
    }
    return false;
}

bool onboard_store_client_device_name(uint8_t client_index, uint8_t* data)
{
    if(MAX_CLIENTS > client_index)
    {
        memcpy((uint8_t*)&client_device_names[client_index], data, BLE_DEVNAME_MAX_LEN);
        APPL_LOG("[OB]: client_device_names[%d] = %s\r\n", client_index, client_device_names[client_index]);
        return true;
    }
    return false;
}

bool onboard_store_passkey_from_wifi(uint8_t passkey_index, uint8_t* data)
{
    memcpy((uint8_t*)&sensors_passkeys[passkey_index], data, DEVICE_PASS_LEN);
    return pstorage_driver_request_store((uint8_t*)(&sensors_passkeys[passkey_index]));
}

bool onboard_store_client_uuid(uint8_t client_index, uint8_t* data)
{
    if(MAX_CLIENTS > client_index)
    {
        memcpy((uint8_t*)&client_device_uuids[client_index], data, DEVICE_UUID_LEN);
        return true;
    }
    return false;
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
            memcpy((uint8_t*)&sensors_passkeys[passkey_index], current_pass, 6);
            return pstorage_driver_request_store((uint8_t*)(&sensors_passkeys[passkey_index]));
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
            memcpy((uint8_t*)&sensors_passkeys[passkey_index], current_pass, 6);
            return pstorage_driver_request_store((uint8_t*)(&sensors_passkeys[passkey_index]));
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
        APPL_LOG("\r\n[OB]:Parse data. State: %d\r\n\r\n", onboard_state);

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
            // if(data[ONBOARD_WIFI_SSID_LEN] == true)
            // {
            //     spi_create_tx_packet(DATA_ID_DEV_CFG_APP, FIELD_ID_CONFIG_WIFI_SSID, OPERATION_WRITE, data, len);
            // }
            // else
            // {
            //     onboard_on_send_complete();
            // }
            break;
        }

        case ONBOARD_STATE_WAIT_WIFI_PASS:
        {
            onboard_state++;
            // if(data[ONBOARD_WIFI_PASS_LEN] == true)
            // {
            //     spi_create_tx_packet(DATA_ID_DEV_CFG_APP, FIELD_ID_CONFIG_WIFI_PASS, OPERATION_WRITE, data, len);
            // }
            // else
            // {
            //     onboard_on_send_complete();
            // }
            break;
        }

        case ONBOARD_STATE_WAIT_MASTER_MODULE_ID:
        {
            onboard_state++;
            // if(data[ONBOARD_MASTER_ID_LEN] == true)
            // {
            //     spi_create_tx_packet(DATA_ID_DEV_CFG_APP, FIELD_ID_CONFIG_MASTER_MODULE_ID, OPERATION_WRITE, data, len);
            // }
            // else
            // {
            //     onboard_on_send_complete();
            // }
            break;
        }

        case ONBOARD_STATE_WAIT_MASTER_MODULE_SEC:
        {
            onboard_state++;
            // if(data[ONBOARD_MASTER_SEC_LEN] == true)
            // {
            //     spi_create_tx_packet(DATA_ID_DEV_CFG_APP, FIELD_ID_CONFIG_MASTER_MODULE_SEC, OPERATION_WRITE, data, ONBOARD_MASTER_SEC_LEN);
            // }
            // else
            // {
            //     onboard_on_send_complete();
            // }
            break;
        }

        case ONBOARD_STATE_WAIT_MASTER_MODULE_URL:
        {
            onboard_state++;
            // if(data[ONBOARD_MASTER_URL_LEN] == true)
            // {
            //     spi_create_tx_packet(DATA_ID_DEV_CFG_APP, FIELD_ID_CONFIG_MASTER_MODULE_URL, OPERATION_WRITE, data, ONBOARD_MASTER_URL_LEN);
            // }
            // else
            // {
            //     onboard_on_send_complete();
            // }
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
    // client_t * p_client;

    if(onboard_state == ONBOARD_STATE_IDLE)
    {
        return;
    }

    // p_client = find_client_by_dev_name(client_device_names[DATA_ID_DEV_CFG_APP], 0);
    // if(
    //     (p_client == NULL) ||
    //     ((p_client->state != STATE_RUNNING) && (p_client->state != STATE_WAIT_READ_RSP))
    //   )
    // {
    //     // Onboarding is started. Waiting device.
    //     if(onboard_get_state() == ONBOARD_STATE_START)
    //     {
    //         return;
    //     }
    //     // Config device is disconnected during onboarding process.
    //     else
    //     {
    //         onboard_set_state(ONBOARD_STATE_ERROR);
    //     }

    // }

    switch(onboard_state)
    {
        case ONBOARD_STATE_IDLE:
        {
            break;
        }

        case ONBOARD_STATE_START:
        {
            APPL_LOG("[OB]: Start config.\r\n");
            // current_char = ONBOARD_CHAR_INDEX_HTU_GYRO_LIGHT_PASS;
            // read_characteristic_value(p_client, ONBOARD_CHAR_UUIDS[current_char++]);
            // onboard_set_state(ONBOARD_STATE_WAIT_HTU_GYRO_LIGHT_PASS);
            break;
        }

        case ONBOARD_STATE_ERROR:
        {
            APPL_LOG("[OB]: Config ERROR.\r\n");
            spi_create_tx_packet(DATA_ID_DEV_CENTRAL, FIELD_ID_CONFIG_ERROR, OPERATION_WRITE, NULL, 0);
            onboard_set_state(ONBOARD_STATE_IDLE);
            break;
        }

        case ONBOARD_STATE_COMPLETE:
        {
            APPL_LOG("[OB]: Config complete.\r\n");
            spi_create_tx_packet(DATA_ID_DEV_CENTRAL, FIELD_ID_CONFIG_COMPLETE, OPERATION_WRITE, NULL, 0);
            onboard_set_state(ONBOARD_STATE_IDLE);
            break;
        }

        default:{}
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

