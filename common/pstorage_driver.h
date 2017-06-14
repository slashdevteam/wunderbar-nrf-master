
/** @file   pstorage_driver.h
 *  @brief  This driver declaration of public functions for working with non-volatile storage 
 *          and corresponding macros, constants,and global variables.
 *
 *  @author MikroElektronika
 *  @bug    No known bugs.
 */

#ifndef _PSTPRAGE_DRIVER_
#define _PSTPRAGE_DRIVER_

/* -- Includes -- */

#include "pstorage.h"
#include "types.h"

/**@brief  Return values for pstorage_driver_load function. */
#define PS_LOAD_STATUS_SUCCESS   0 
#define PS_LOAD_STATUS_FAIL      1 
#define PS_LOAD_STATUS_EMPTY     2
#define PS_LOAD_STATUS_NOT_FOUND 3

/**@brief  Possible error codes for storing process. */
#define PS_STORE_STATUS_NO_ERR          0
#define PS_STORE_STATUS_ERR_CLEAR_MAGIC 1
#define PS_STORE_STATUS_ERR_STORE_DATA  2
#define PS_STORE_STATUS_ERR_ADD_MAGIC   3

/**@brief  Type of function which initialize and configure pstorage, and registers characteristic values to corresponding blocks in persistent memory..*/
typedef bool (*pstorage_driver_init_t)(void);

/** @brief  Configure pstorage driver.
 *
 *  @param  block_size Desired block size for persistent memory storage
 *
 *  @return false in case error occurred, otherwise true.
 */
bool     pstorage_driver_cfg(uint16_t block_size);

/** @brief  This function is called to initialize fields of current block of pstorage_driver record.
 *
 *  @param  data  Pointer to data which will be related to current block.
 *  @param  size  Size of data in bytes.
 *
 *  @return false in case error occurred, otherwise true.
 */
bool     pstorage_driver_register_block(uint8_t * data, uint16_t size);

/** @brief  This function started storing data to persistent memory.
 *
 *  @param  data  Pointer to buffer which need to be stored.
 *
 *  @return false in case error occurred, otherwise true.
 */
bool     pstorage_driver_request_store(uint8_t * source_data);

/** @brief  This function is called to load data from persistent memory.
 *
 *  @param  data  Pointer to destination buffer.
 *
 *  @return PS_LOAD_STATUS_SUCCESS, PS_LOAD_STATUS_FAIL, PS_LOAD_STATUS_EMPTY, PS_LOAD_STATUS_NOT_FOUND
 */
uint32_t pstorage_driver_load(uint8_t * dest_data);

/** @brief  Get error status of storing process and clear error status.
 *
 *  @return PS_STORE_STATUS_NO_ERR, PS_STORE_STATUS_ERR_CLEAR_MAGIC, PS_STORE_STATUS_ERR_STORE_DATA, PS_STORE_STATUS_ERR_ADD_MAGIC
 */
uint32_t pstorage_driver_get_store_status(void);

/** @brief  Get storing process state.
 *
 *  @return true if storing is in progress, otherwise false.
 */
bool     pstorage_driver_get_run_status(void);

/** @brief  This function handles various states in storing process.
 *
 *  @return Void.
 */
void     pstorage_driver_run(void);

#endif /* _PSTPRAGE_DRIVER_ */
