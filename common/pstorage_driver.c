
/** @file   pstorage_driver.c
 *  @brief  This driver contains functions for working with non-volatile storage 
 *          and corresponding macros, constants,and global variables.
 *
 *  @author MikroElektronika
 *  @bug    No known bugs.
 */
 
/* -- Includes -- */

#include "pstorage_driver.h"
#include "nrf_error.h"
#include "nrf_soc.h"
#include "onboard.h"
#include <string.h>

#define PSTORAGE_DRIVER_MAGIC_NUM         0x45DEAAAA  /**< Value which will be written at the end of block in persistent memory. Used to check validity of store operation. */
#define PSTORAGE_DRIVER_NUM_OF_BLOCKS     7           /**< Number of blocks requested by the module. */
#define PSTORAGE_NUMBER_OF_STORE_STATES   4           /**< Number of states in storing process. */

/**@brief  This record used to identify block into persistent memory by address of buffer RAM. */
typedef struct 
{
    uint8_t *          data;                                         /**< Pointer to data buffer. */
    uint16_t           size;                                         /**< Size of data buffer. */
    pstorage_handle_t  block_id;                                     /**< Block identifier to identify persistent memory blocks. */
} 
pstorage_driver_block_t;

/**@brief  This record contain properties which are used during registration of persistent storage interface, and list of blocks. */
typedef struct 
{
    pstorage_module_param_t   module_param;                          /**< Module registration param. */
    pstorage_handle_t         base_id;                               /**< Base block identifier. */
    pstorage_driver_block_t   block[PSTORAGE_DRIVER_NUM_OF_BLOCKS];  /**< List of blocks. */
} 
pstorage_driver_t;

/**@brief  Storing process states. */
typedef enum
{
    STORE_STATE_IDLE,                                                /**< Idle state. */ 
    STORE_STATE_CLEAR_MAGIC,                                         /**< Clearing magic number. */  
    STORE_STATE_STORE_DATA,                                          /**< Storing data. */  
    STORE_STATE_ADD_MAGIC                                            /**< Storing magic number. */  
} 
pstorage_driver_store_state_t;

/**@brief  Storing process record */
typedef struct 
{
    pstorage_driver_store_state_t state;                             /**< Current state of storing process. */
    pstorage_driver_block_t *     block;                             /**< Current storing block. */
    uint32_t                      error_status;                      /**< Error status of process. */
    bool                          run_flag;                          /**< Indicates whether process is in running state or not. */
    bool                          wait_flag;                         /**< Indicates whether currently waiting for pstorage event. */
} 
pstorage_driver_store_t;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**@brief Declaration of static variables. */

static pstorage_driver_t pstorage_driver;
static pstorage_driver_store_t  pstorage_driver_store;
static uint32_t         tmp_magic_number;
static uint16_t         num_of_reg_blocks = 0;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**@brief Declaration of static functions. */

static void pstorage_driver_set_next_state(void);
static void pstorage_driver_set_idle_state(void);
static void pstorage_driver_update_store_status(void);
static pstorage_driver_block_t * pstorage_driver_get_block(uint8_t * data);
static void pstorage_driver_cb_handler(pstorage_handle_t * handle, uint8_t op_code, uint32_t result, uint8_t * p_data, uint32_t data_len);

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** @brief  Configure pstorage driver.
 *
 *  @param  block_size Desired block size for persistent memory storage
 *
 *  @return  false in case that error is occurred, otherwise true.
 */

bool pstorage_driver_cfg(uint16_t block_size) 
{
    uint32_t err_code;
   
    // Set module registration param.	
    pstorage_driver.module_param.block_size  = block_size;                     // Set desired block size for persistent memory storage.  
    pstorage_driver.module_param.block_count = PSTORAGE_DRIVER_NUM_OF_BLOCKS;  // Set number of blocks requested by the module. 
    pstorage_driver.module_param.cb          = pstorage_driver_cb_handler;     // Set persistent storage error reporting callback
    
	  // Initialize fields for store opperation.
    pstorage_driver_store.block        = NULL;
    pstorage_driver_store.error_status = PS_STORE_STATUS_NO_ERR;
    pstorage_driver_store.state        = STORE_STATE_IDLE;
    pstorage_driver_store.run_flag     = false;
    pstorage_driver_store.wait_flag    = false;
    
	  // Register with persistent storage interface.
    err_code = pstorage_register(&pstorage_driver.module_param, &pstorage_driver.base_id);
    return (err_code == NRF_SUCCESS) ? true : false;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** @brief  This function is called to initialize fields of current block of pstorage_driver record.
 *
 *  @param  data  Pointer to data which will be related to current block.
 *  @param  size  Size of data in bytes.
 *
 *  @return  false in case that error is occurred, otherwise true.
 */

bool pstorage_driver_register_block(uint8_t * data, uint16_t size) 
{
    uint32_t err_code;
    
    pstorage_driver.block[num_of_reg_blocks].data = data;  // Set data field of current block (which is determined by value of num_of_reg_blocks).
    pstorage_driver.block[num_of_reg_blocks].size = size;  // Set size field of current block (which is determined by value of num_of_reg_blocks).
    
	  // Function to get block id with reference to base block id and current block (which is determined by value of num_of_reg_blocks).
    err_code = pstorage_block_identifier_get(&pstorage_driver.base_id, num_of_reg_blocks, &pstorage_driver.block[num_of_reg_blocks].block_id);
    if (err_code != NRF_SUCCESS) 
    {
        return false;
    }
		
		// Incerment value of num_of_reg_blocks.
    num_of_reg_blocks++;
    return true;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** @brief  This function is called to load data from persistent memory.
 *
 *  @param  data  Pointer to destination buffer.
 *
 *  @return PS_LOAD_STATUS_SUCCESS, PS_LOAD_STATUS_FAIL, PS_LOAD_STATUS_EMPTY, PS_LOAD_STATUS_NOT_FOUND
 */

uint32_t pstorage_driver_load(uint8_t * dest_data) 
{
    uint32_t err_code;
    uint16_t tmp_size;
    pstorage_driver_block_t * block;
   
    // Get pstorage_driver block, based on address of data.	
    block = pstorage_driver_get_block(dest_data);
    if(block == NULL) 
    {
        return PS_LOAD_STATUS_NOT_FOUND;                                       // Block with corresponding data not registered.
    }
    
    // Load persistently stored data.
    err_code = pstorage_load(dest_data, &block->block_id, block->size, 0);
    if(err_code != NRF_SUCCESS) 
    {
        return PS_LOAD_STATUS_FAIL;
    }
    
		// Need to check is the last 4 bytes of corresponding block equal to PSTORAGE_DRIVER_MAGIC_NUM.
		
		// If block->size is divisible by 4, PSTORAGE_DRIVER_MAGIC_NUM is located on offset block->size.
    tmp_size = block->size;
		
    if((tmp_size % 4) != 0) 
    {
			  // In case that block->size is not divisible by 4, offset for PSTORAGE_DRIVER_MAGIC_NUM is equal with first larger  number that is divisible by 4.
        tmp_size += 4 - (tmp_size % 4);
    }
    
		// Load persistently stored data. 
    err_code = pstorage_load((uint8_t *)&tmp_magic_number, &block->block_id, 4, tmp_size);
    if(err_code != NRF_SUCCESS) 
    {
        return PS_LOAD_STATUS_FAIL;
    }
    else if(tmp_magic_number != PSTORAGE_DRIVER_MAGIC_NUM) 
    {
        return PS_LOAD_STATUS_EMPTY;                                           // Can be considered that the corresponding block is empty.
    }
    
    return PS_LOAD_STATUS_SUCCESS;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** @brief  Get error status of storing process and clear error status.
 *
 *  @return PS_STORE_STATUS_NO_ERR, PS_STORE_STATUS_ERR_CLEAR_MAGIC, PS_STORE_STATUS_ERR_STORE_DATA, PS_STORE_STATUS_ERR_ADD_MAGIC
 */

uint32_t pstorage_driver_get_store_status(void) 
{
    uint32_t tmp_status;
    tmp_status = pstorage_driver_store.error_status;
    pstorage_driver_store.error_status = PS_STORE_STATUS_NO_ERR;
    return tmp_status;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** @brief  This function started storing data to persistent memory.
 *
 *  @param  data  Pointer to buffer which need to be stored.
 *
 *  @return  false in case that error is occurred, otherwise true.
 */

bool pstorage_driver_request_store(uint8_t * source_data) 
{
    pstorage_driver_block_t * block;
    
    // Return if storing is already in progress.	
    if(pstorage_driver_store.run_flag == true) 
    {
        return false;
    }
    
		// Get pstorage_driver block, based on address of data.
    block = pstorage_driver_get_block(source_data);
    if(block == NULL) 
    {
        return false;
    }
    
    pstorage_driver_store.block = block;
    pstorage_driver_store.run_flag = true;
    
    return true;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** @brief  This function handles various states in storing process.
 *
 *  @return Void.
 */

void pstorage_driver_run(void) 
{
    uint32_t err_code;

    // Check whether the currently waiting for pstorage event.	
    if(pstorage_driver_store.wait_flag) 
    {
        return;
    } 
    
    switch(pstorage_driver_store.state) 
    {
      
        // Idle state
        case STORE_STATE_IDLE: 
        {
					  // Check whether the storing process is running.
            if(!pstorage_driver_store.run_flag) 
            {
                return;
            }
						
            pstorage_driver_set_next_state();                                  // Go to next state.
        }
        
				// Clear PSTORAGE_DRIVER_MAGIC_NUM from corresponding block.
        case STORE_STATE_CLEAR_MAGIC: 
        {
            uint16_t tmp_size;
					
					  // Calculate offset of PSTORAGE_DRIVER_MAGIC_NUM in block.
            tmp_size = pstorage_driver_store.block->size;
            if((tmp_size % 4) != 0) 
            {
                tmp_size += 4 - (tmp_size % 4);
            }
						
						// Start storing "clear" value.
            tmp_magic_number = 0xFFFFFFFF;
            err_code = pstorage_update(&pstorage_driver_store.block->block_id, (uint8_t *)&tmp_magic_number, 4, tmp_size);
            if(err_code != NRF_SUCCESS) 
            {
							  // Stop storing process.
                pstorage_driver_update_store_status();
                pstorage_driver_set_idle_state();
                return;
            }
            pstorage_driver_store.wait_flag = true;
            break;
        }
          
				// Store data.
        case STORE_STATE_STORE_DATA: 
        {
            uint16_t tmp_size;
					
					  // Size of storing data must be divisible by 4.
            tmp_size = pstorage_driver_store.block->size;
            if((tmp_size % 4) != 0) 
            {
                tmp_size += 4 - (tmp_size % 4);
            } 
						
						// Start storing data.
            err_code = pstorage_update(&pstorage_driver_store.block->block_id, pstorage_driver_store.block->data, tmp_size, 0);
            if(err_code != NRF_SUCCESS) 
            {
							  // Stop storing process.
                pstorage_driver_update_store_status();
                pstorage_driver_set_idle_state();
                return;
            }
            pstorage_driver_store.wait_flag = true;
            break;
        }
        
				// Store PSTORAGE_DRIVER_MAGIC_NUM at the end of data.
        case STORE_STATE_ADD_MAGIC: 
        {
            uint16_t tmp_size;
					  
					  // Calculate offset of PSTORAGE_DRIVER_MAGIC_NUM in block.
            tmp_size = pstorage_driver_store.block->size;
            if((tmp_size % 4) != 0) 
            {
                tmp_size += 4 - (tmp_size % 4);
            }
            tmp_magic_number = PSTORAGE_DRIVER_MAGIC_NUM;
						// Start storing PSTORAGE_DRIVER_MAGIC_NUM value.
            err_code = pstorage_update(&pstorage_driver_store.block->block_id, (uint8_t *)&tmp_magic_number, 4, tmp_size);
            if(err_code != NRF_SUCCESS) 
            {
							  // Stop storing process.
                pstorage_driver_update_store_status();
                pstorage_driver_set_idle_state();
                return;
            }
            pstorage_driver_store.wait_flag = true;
            break;
        }     
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** @brief  Get storing process state.
 *
 *  @return true if storing is in progress, otherwise false.
 */

bool pstorage_driver_get_run_status(void) 
{
    return pstorage_driver_store.run_flag;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** @brief  Function to be called to go to next storing state.
 *
 *  @return Void.
 */

static void pstorage_driver_set_next_state(void) 
{
	  // Calculate value of next state.
    pstorage_driver_store.state = (pstorage_driver_store_state_t)((uint8_t)(pstorage_driver_store.state + 1) % PSTORAGE_NUMBER_OF_STORE_STATES);
  
	  // If changed state is STORE_STATE_IDLE update value of run_flag field.
    if(pstorage_driver_store.state == STORE_STATE_IDLE) 
    {
        pstorage_driver_store.run_flag = false;
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** @brief  Function to be called to set STORE_STATE_IDLE of storing process.
 *
 *  @return Void.
 */

static void pstorage_driver_set_idle_state(void) 
{
    pstorage_driver_store.state = STORE_STATE_IDLE;
    pstorage_driver_store.run_flag = false;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** @brief  Function to be called to set error status of currnet storing process. This function saves state in which error occurred.
 *
 *  @return Void.
 */

static void pstorage_driver_update_store_status(void) 
{
    pstorage_driver_store.error_status = pstorage_driver_store.state;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** @brief  Get pstorage_driver block, based on address of data.
 *
 *  @return Corresponding pstorage_driver block. NULL if there is not such block.
 */

static pstorage_driver_block_t * pstorage_driver_get_block(uint8_t * data) 
{
    uint16_t cnt;
	
	  // Search if there is a block with data field matching the input parameter.
    for(cnt = 0; cnt < PSTORAGE_DRIVER_NUM_OF_BLOCKS; cnt++) 
    {
        if(pstorage_driver.block[cnt].data == data) 
        {
            return (&pstorage_driver.block[cnt]);
        }
    }
    return NULL;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** @brief  Set persistent storage event reporting callback.
 *
 *  @param  handle    Identifies module and block for which callback is received.
 *  @param  op_code   Identifies the operation for which the event is notified.
 *  @param  result    Identifies the result of flash access operation. NRF_SUCCESS implies, operation succeeded.
 *  @param  p_data    Identifies the application data pointer.
 *  @param  data_len  Length data application had provided for the operation.
 *
 *  @return Void.
 */

static void pstorage_driver_cb_handler(pstorage_handle_t * handle, uint8_t op_code, uint32_t result, uint8_t * p_data, uint32_t data_len)
{
  switch(op_code) {
		
		// Update an already touched storage block.
        case PSTORAGE_UPDATE_OP_CODE:
        {
            if (result == NRF_SUCCESS)
            {
							   // Go to next state.
                pstorage_driver_set_next_state();
							  // Check if there is reached idle state of storing process, and cleared run_flag.
                if(pstorage_driver_get_run_status() == false)
                {
                    onboard_on_store_complete(); 
                }
            }
            else 
            {
							  // Stop storing process.
                pstorage_driver_update_store_status();
                pstorage_driver_set_idle_state();
            }
            pstorage_driver_store.wait_flag = false;
            break;
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
