#include <avr/io.h>
#include <stddef.h>

// Definitions ------------------------------------------------------
#define MOTOR_DOOR_PIN_UP PD7
#define MOTOR_DOOR_PIN_DOWN PD6
#define MOTOR_FEEDER_PIN PD5

#define LED_BATTERY_PIN PB1
#define LED_STATUS_PIN PB0

#define ADC_BATTERY 0
#define ADC_LIGHT 1

#define STATE_MOVING_UP 0
#define STATE_MOVING_DOWN 1
#define STATE_STATIONARY_UP 2
#define STATE_STATIONARY_DOWN 3
#define STATE_RELEASE_UPPER_SWITCH 4
#define STATE_RELEASE_LOWER_SWITCH 5
#define STATE_ERROR 6

// Global variables ------------------------------------------------------
uint8_t state = STATE_STATIONARY_UP;

// Global methods ------------------------------------------------------


// main ------------------------------------------------------
int main() {
    // init outputs for LED / motors
    DDRB |= (1 << LED_BATTERY_PIN) | (1 << LED_STATUS_PIN);
    DDRD |= (1 << MOTOR_DOOR_PIN_DOWN) | (1 << MOTOR_DOOR_PIN_UP) | (1 << MOTOR_FEEDER_PIN);

    // main loop ------------------------------------------------------
    while(1) {

    }

    return 0;
}