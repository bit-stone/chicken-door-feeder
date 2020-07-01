#include <avr/io.h>
#include <stddef.h>

// Definitions ------------------------------------------------------
#define MOTOR_DOOR_PIN_UP PD7
#define MOTOR_DOOR_PIN_DOWN PD6
#define MOTOR_FEEDER_PIN PD5

#define MOTOR_DOOR_ENABLE_PIN PB1

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

#define MOTOR_DOOR_DIRECTION_UP 10
#define MOTOR_DOOR_DIRECTION_DOWN 11
#define MOTOR_DOOR_STOP 12

// Global variables ------------------------------------------------------
uint8_t state = STATE_STATIONARY_UP;
uint8_t inMovement = 0;

// Global methods ------------------------------------------------------

// enable door motor h-bridge
void enableDoorMotor() {
    PORTB |= (1 << MOTOR_DOOR_ENABLE_PIN);
}

// disable door motor h-brigde
void disableDoorMotor() {
    PORTB &= ~(1 << MOTOR_DOOR_ENABLE_PIN);
}

// set door motor direction or stop it
void setDoorMotor(uint8_t direction) {
    switch(direction) {
        case MOTOR_DOOR_DIRECTION_UP:
            inMovement = 1;
            enableDoorMotor();
            PORTD &= ~(1 << MOTOR_DOOR_DIRECTION_DOWN);
            PORTD |= (1 << MOTOR_DOOR_DIRECTION_UP);
        break;
        case MOTOR_DOOR_DIRECTION_DOWN:
            inMovement = 1;
            enableDoorMotor();
            PORTD |= (1 << MOTOR_DOOR_DIRECTION_DOWN);
            PORTD &= ~(1 << MOTOR_DOOR_DIRECTION_UP);
            break;
        case MOTOR_DOOR_STOP:
            inMovement = 0;
            disableDoorMotor();
            PORTD &= ~(1 << MOTOR_DOOR_DIRECTION_UP);
            PORTD &= ~(1 << MOTOR_DOOR_DIRECTION_DOWN);
            break;
    }
}

void setState(uint8_t newState) {
    if(newState != state) {
        switch(newState) {
            case STATE_MOVING_UP:
                setDoorMotor(MOTOR_DOOR_DIRECTION_UP);
            break;
            case STATE_MOVING_DOWN:
                setDoorMotor(MOTOR_DOOR_DIRECTION_DOWN);
            break;
            case STATE_STATIONARY_UP:
                setDoorMotor(MOTOR_DOOR_STOP);
            break;
            case STATE_STATIONARY_DOWN:
                setDoorMotor(MOTOR_DOOR_STOP);
            break;
            case STATE_RELEASE_UPPER_SWITCH:
                setDoorMotor(MOTOR_DOOR_DIRECTION_DOWN);
            break;
            case STATE_RELEASE_LOWER_SWITCH:
                setDoorMotor(MOTOR_DOOR_DIRECTION_UP);
            break;
            case STATE_ERROR:
                setDoorMotor(MOTOR_DOOR_STOP);
            break;
        }
    }
}

// main ------------------------------------------------------
int main() {
    // init outputs for LED
    DDRB |= (1 << LED_BATTERY_PIN) | (1 << LED_STATUS_PIN);

    // init outputs for motor
    DDRD |= (1 << MOTOR_DOOR_PIN_DOWN) | (1 << MOTOR_DOOR_PIN_UP) | (1 << MOTOR_FEEDER_PIN);
    DDRB |= (1 << MOTOR_DOOR_ENABLE_PIN);


    // initially move up
    setState(STATE_MOVING_UP);
    
    // main loop ------------------------------------------------------
    while(1) {

    }

    return 0;
}