#include <avr/io.h>

// where to find the status LED
#define LED_STATUS_PIN PD4
#define LED_STATUS_PORT PORTD
#define LED_STATUS_DDR DDRD

// simple functions
void led_init();
void led_on();
void led_off();
void led_toggle();