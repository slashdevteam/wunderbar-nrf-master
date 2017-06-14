
/** @file   gpio.h
 *  @brief  GPIO driver.
 *
 *  This contains the driver declarations for the GPIO module.
 *
 *  @author MikroElektronika
 *  @bug    No known bugs.
 */

#ifndef _NRF51822_GPIO_
#define _NRF51822_GPIO_

#include "types.h"


typedef struct {
/* TASKS */
/* EVENTS */
/* REGISTERS */
/* DEVICE REGISTERS */
  HW_UU unused1[0x141]; // padding
  HW_RW OUT;            // Write entire GPIO port.
  HW_RW OUTSET;         // Set individual bits in GPIO port.
  HW_RW OUTCLR;         // Clear individual bits in GPIO port.
  HW_RO IN;             // Read GPIO port.
  HW_RW DIR;            // Direction of GPIO pins. (1 = out, 0 = in)
  HW_RW DIRSET;         // DIR set register. (1 = out)
  HW_RW DIRCLR;         // DIR clear register. (1 = in)
  HW_UU unused2[0x78];  // padding
  HW_RW PIN_CNF[0x20];  // Configuration of GPIO pins.
} GPIO_STRUCT;


#define GPIO0 ((GPIO_STRUCT*)(0x50000000))

// PIN_CNF meaning, bits:
// 0      | DIR    | 0 = in 1 = out
// 1      | INPUT  | 0 = connect 1 = disconnect | input buffer
// 2,3    | PULL   | 0 = no, 1 = down, 3 = up
// 4..7   | RESV
// 8,9,10 | DRIVE  | 
// 16,17  | SENSE  | 0 = disabled, 2 = high, 3 = low


// GPIO API types and functions

typedef enum {
  PIN_PULL_NONE   = 0,
  PIN_PULL_DOWN = 1,
  PIN_PULL_UP   = 3
} PIN_PULL;

typedef enum {
  PIN_DRIVE_S0S1 = 0,
  PIN_DRIVE_H0S1 = 1, 
  PIN_DRIVE_S0H1 = 2,
  PIN_DRIVE_H0H1 = 3,
  PIN_DRIVE_D0S1 = 4,
  PIN_DRIVE_D0H1 = 5,
  PIN_DRIVE_S0D1 = 6,
  PIN_DRIVE_H0D1 = 7
} PIN_DRIVE;

typedef enum {
  PIN_SENSE_DISABLED = 0,
  PIN_SENSE_HIGH     = 2,
  PIN_SENSE_LOW      = 3
} PIN_SENSE;

/** @brief Function sets pin to be digital input
 *
 *  @param pin        Number of pin.
 *  @param pull_mode  Selecting the pin to be pulled down or up.
 *
 *  @return Void.
 */
void gpio_set_pin_digital_input(uint8_t pin, PIN_PULL pull_mode);

/** @brief Function sets pin to be digital output
 *
 *  @param pin        Number of pin.
 *  @param pull_mode  Selecting drive mode of pin.
 *
 *  @return Void.
 */
void gpio_set_pin_digital_output(uint8_t pin, PIN_DRIVE drive_mode);

/** @brief Function connect or disconnect input buffer.
 *
 *  @param pin        Number of pin.
 *
 *  @return Void.
 */
void gpio_disconnect_pin(uint8_t pin);

/** @brief Function read state of corresponding input pin.
 *
 *  @param pin    Number of pin.
 *
 *  @return Void.
 */
bool gpio_read(uint8_t pin);

/** @brief Function set state of output pin.
 *
 *  @param pin    Number of pin.
 *  @param value  New state.
 *
 *  @return Void.
 */
void gpio_write(uint8_t pin, bool value);

#endif /* _NRF51822_GPIO_ */
