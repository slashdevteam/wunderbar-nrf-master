/*
 * Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is confidential property of Nordic Semiconductor. The use,
 * copying, transfer or disclosure of such information is prohibited except by express written
 * agreement with Nordic Semiconductor.
 *
 */

 /**@file
 *
 * @defgroup XXXX
 * @{
 * @ingroup  YYYY
 *
 * @brief    ZZZZZ.
 */

#ifndef CLIENT_HANDLING_H__
#define CLIENT_HANDLING_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_db_discovery.h"
#include "device_manager.h"
#include "device_manager_cnfg.h"
#include "wunderbar_common.h"
#include "onboard.h"

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**@brief Client states. */

typedef enum
{
    STATE_SERVICE_DISC         = 0,    // Service discovery state.
    STATE_DEVICE_IDENTIFYING   = 1,    // Check Wunderbar ID.
    STATE_NOTIF_ENABLE         = 2,    // State where the request to enable notifications is sent to the peer.
    STATE_RUNNING              = 3,    // Running state.
    STATE_WAIT_READ_RSP        = 4,    // Wait for read response.
    STATE_WAIT_WRITE_RSP       = 5,    // Wait for write response.
    STATE_DISCONNECTING        = 6,    // Disconnect request is sent.
    STATE_IDLE                 = 7,    // Idle state.
    STATE_ERROR                = 8,    // Error state.
    STATE_CONFIGURE            = 9,    // Sensor under configuration
    STATE_CHECK_CONFIG         = 10    // Validating config
}
client_state_t;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**@brief Advertised data record. */

typedef struct
{
    const uint8_t * device_name;
    ble_gap_addr_t  peer_addr;
	  bool            bonded_flag;
}
current_conn_device_t;


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**@brief Client context information. */

typedef struct
{
    ble_db_discovery_t    srv_db;            /**< The DB Discovery module instance associated with this client. */
    dm_handle_t           handle;            /**< Device manager identifier for the device. */
    const uint8_t *       device_name;       /**< Client Device Name. */
    ble_gap_addr_t        peer_addr;         /**< Bluetooth Low Energy address. */
    sensorID_t            id;                /**< Bluetooth Low Energy address. */
    uint8_t               state;             /**< Client state. */
    uint8_t               srv_index;         /**< These two fields determine last found characteristic with notification properties. Used to enable services. */
    uint8_t               char_index;        /**<                                                                                                             */
}
client_t;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**@brief Functions declarations. */
bool read_characteristic_value(client_t * p_client, uint16_t uuid);
client_t * find_client_by_dev_name(const uint8_t * device_name, uint8_t len);
ble_db_discovery_char_t * find_char_by_uuid(uint16_t char_uuid, client_t * p_client);
ble_db_discovery_char_t * find_char_by_handle_value(uint16_t handle_value, client_t * p_client);
bool write_characteristic_value(client_t * p_client, uint16_t uuid, uint8_t * data, uint16_t len);
bool write_char_value(client_t * p_client, uint16_t uuid, uint8_t * data, uint16_t len);
uint16_t search_for_client_configuring(void);
bool check_client_state(uint16_t state, uint16_t index);
void search_for_client_event(void);
void ignore_list_add(ble_gap_addr_t * p_peer_addr);
bool ignore_list_search(ble_gap_addr_t * p_peer_addr);
void scan_stop(void);
void scan_start(void);
bool validate_device_name(uint8_t * device_name, uint16_t len, const uint8_t ** found_device_name);
void check_client_timeout(void);
bool timers_init(void);
uint8_t get_active_client_number(void);

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**@brief Funtion for initializing the module.
 */

void client_handling_init(onboard_mode_t onboard_mode);

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**@brief Funtion for returning the current number of clients.
 *
 * @return  The current number of clients.
 */

uint8_t client_handling_count(void);

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**@brief Funtion for creating a new client.
 *
 * @param[in] p_handle    Device Manager Handle. For link related events, this parameter
 *                        identifies the peer.
 *
 * @param[in] conn_handle Identifies link for which client is created.
 * @return NRF_SUCCESS on success, any other on failure.
 */

uint32_t client_handling_create(const dm_handle_t * p_handle, uint16_t conn_handle, current_conn_device_t * current_conn_device);

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**@brief Funtion for freeing up a client by setting its state to idle.
 *
 * @param[in] p_handle  Device Manager Handle. For link related events, this parameter
 *                      identifies the peer.
 *
 * @return NRF_SUCCESS on success, any other on failure.
 */

uint32_t client_handling_destroy(const dm_handle_t * p_handle);

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**@brief Funtion for handling client events.
 *
 * @param[in] p_ble_evt  Event to be handled.
 */

void client_handling_ble_evt_handler(ble_evt_t * p_ble_evt);

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**@brief Funtion for handling device manager events.
 *
 * @param[in] p_handle       Identifies device with which the event is associated.
 * @param[in] p_event        Event to be handled.
 * @param[in] event_result   Event result indicating whether a procedure was successful or not.
 */

api_result_t client_handling_dm_event_handler(const dm_handle_t    * p_handle,
                                              const dm_event_t     * p_event,
                                              const api_result_t     event_result);

#endif // CLIENT_HANDLING_H__

/** @} */
