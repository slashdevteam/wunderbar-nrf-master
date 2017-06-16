#define LEDS_NUMBER 1
#define LED_0 29

#define BUTTONS_NUMBER 1
#define SIGNAL_BOOTLOADER_ENTER 4
#define BUTTON_PULL NRF_GPIO_PIN_PULLUP

#define RX_PIN_NUMBER 3
#define TX_PIN_NUMBER 5
#define RTS_PIN_NUMBER -1
#define CTS_PIN_NUMBER -1

enum {
        BSP_INDICATE_FATAL_ERROR,
};

static inline void bsp_indication_set(int indication) {}
