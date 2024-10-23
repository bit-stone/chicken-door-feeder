#include "led.h"

void led_init()
{
    // set data direction register as output and set output low by default
    LED_STATUS_DDR |= (1 << LED_STATUS_PIN);
    led_off();
}

void led_on()
{
    LED_STATUS_PORT |= (1 << LED_STATUS_PIN);
}

void led_off()
{
    LED_STATUS_PORT &= ~(1 << LED_STATUS_PIN);
}

void led_toggle()
{
    LED_STATUS_PORT ^= (1 << LED_STATUS_PIN);
}