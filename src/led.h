#include <avr/io.h>

// where to find the status LED
#define LED_STATUS_PIN PD4
#define LED_STATUS_PORT PORTD
#define LED_STATUS_DDR DDRD

// where to find debug led
#define LED_DEBUG_PIN PB7
#define LED_DEBUG_PORT PORTB
#define LED_DEBUG_DDR DDRB

// simple functions
void led_init();
void led_on();
void led_off();
void led_toggle();

// debug led
void led_debug_init();
void led_debug_on();
void led_debug_off();
void led_debug_toggle();