
/** @file   spi_slave_config.c
 *  @brief  This driver contains functions for working with spi module 
 *          and corresponding macros, constants,and global variables.
 *
 *  @author MikroElektronika
 *  @bug    No known bugs.
 */

#ifndef SPI_SLAVE_EXAMPLE_H__
#define SPI_SLAVE_EXAMPLE_H__

#include <stdint.h>
#include "wunderbar_common.h"

/**@brief Function for initializing the SPI slave example.
 *
 * @retval NRF_SUCCESS  Operation success.
 */ 
bool spi_slave_app_init(void);

void spi_create_tx_packet(data_id_t data_id_t, uint8_t field_id, uint8_t operation, uint8_t * data, uint8_t len);
void spi_lock_tx_packet(data_id_t data_id);
void spi_check_tx_ready(void);
void spi_clear_tx_packet(data_id_t data_id);
bool spi_search_full_frame(void);

#endif // SPI_SLAVE_EXAMPLE_H__

/** @} */
