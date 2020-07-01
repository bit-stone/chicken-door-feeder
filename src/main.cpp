#include <avr/io.h>
#include <stddef.h>
#include <avr/interrupt.h>
#include <util/delay.h>

// MAX Settings
#define MAX_MOVEMENT_COUNTER 360 // about 180 seconds
#define FEEDER_DURATION_MS 1000
#define FEEDER_DELAY_COUNT 10

// Definitions ------------------------------------------------------
#define MOTOR_DOOR_PIN_UP PD7
#define MOTOR_DOOR_PIN_DOWN PD6
#define MOTOR_FEEDER_PIN PD5

#define MOTOR_DOOR_ENABLE_PIN PB2

#define UPPER_SWITCH_PIN PD2
#define LOWER_SWITCH_PIN PD3

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

#define WDT_16MS 64   //    0        1         0         0        0        0          0          0
#define WDT_32MS 65  //    0        1         0         0        0        0          0          1
#define WDT_64MS 66   //    0        1         0         0        0        0          1          0
#define WDT_125MS 67  //    0        1         0         0        0        0          1          1
#define WDT_250MS 68  //    0        1         0         0        0        0          0          0
#define WDT_500MS 69  //    0        1         0         0        0        1          0          0
#define WDT_1000MS 70   //    0        1         0         0        0        1          0          1
#define WDT_2000MS 71 //    0        1         0         0        0        1          1          1
#define WDT_4000MS 96    //    0        1         1         0        0        0          0          0
#define WDT_8000MS 97

volatile uint8_t state = STATE_STATIONARY_UP;
volatile uint8_t inMovement = 0;

uint8_t releaseCounter = 0;
uint16_t movementCounter = 0;
uint16_t feederCounter = 0;

uint8_t watchDogTimerFlag = 0;

// Global methods ------------------------------------------------------

// enable door motor h-bridge
void enableDoorMotor() {
    PORTB |= (1 << MOTOR_DOOR_ENABLE_PIN);
}

// disable door motor h-brigde
void disableDoorMotor() {
    PORTB &= ~(1 << MOTOR_DOOR_ENABLE_PIN);
}

void setWatchdogTimer(uint8_t input)
{
    cli();
    MCUSR &= ~(1 << WDRF);
    WDTCSR = (1 << WDCE) | (1 << WDE);
    WDTCSR = input;
    sei();
}

// set door motor direction or stop it
void setDoorMotor(uint8_t direction) {
    switch(direction) {
        case MOTOR_DOOR_DIRECTION_UP:
            inMovement = 1;
            enableDoorMotor();
            PORTD &= ~(1 << MOTOR_DOOR_PIN_DOWN);
            _delay_ms(50);
            PORTD |= (1 << MOTOR_DOOR_PIN_UP);
        break;
        case MOTOR_DOOR_DIRECTION_DOWN:
            inMovement = 1;
            enableDoorMotor();
            PORTD &= ~(1 << MOTOR_DOOR_PIN_UP);
            _delay_ms(50);
            PORTD |= (1 << MOTOR_DOOR_PIN_DOWN);
        break;
        case MOTOR_DOOR_STOP:
            inMovement = 0;
            disableDoorMotor();
            PORTD &= ~(1 << MOTOR_DOOR_PIN_DOWN);
            PORTD &= ~(1 << MOTOR_DOOR_PIN_UP);
        break;
    }
}

void setState(uint8_t newState) {
    cli();
    if(newState != state) {
        switch(newState) {
            case STATE_MOVING_UP:
                setWatchdogTimer(WDT_500MS);
                setDoorMotor(MOTOR_DOOR_DIRECTION_UP);
            break;
            case STATE_MOVING_DOWN:
                setWatchdogTimer(WDT_500MS);
                setDoorMotor(MOTOR_DOOR_DIRECTION_DOWN);
            break;
            case STATE_STATIONARY_UP:
                setWatchdogTimer(WDT_8000MS);
                setDoorMotor(MOTOR_DOOR_STOP);
                PORTB &= ~(1 << LED_STATUS_PIN);
            break;
            case STATE_STATIONARY_DOWN:
                setWatchdogTimer(WDT_8000MS);
                setDoorMotor(MOTOR_DOOR_STOP);
                PORTB &= ~(1 << LED_STATUS_PIN);
            break;
            case STATE_RELEASE_UPPER_SWITCH:
                setWatchdogTimer(WDT_500MS);
                setDoorMotor(MOTOR_DOOR_DIRECTION_DOWN);
            break;
            case STATE_RELEASE_LOWER_SWITCH:
                setWatchdogTimer(WDT_500MS);
                setDoorMotor(MOTOR_DOOR_DIRECTION_UP);
            break;
            case STATE_ERROR:
                setWatchdogTimer(WDT_8000MS);
                setDoorMotor(MOTOR_DOOR_STOP);
            break;
        }
        state = newState;
        movementCounter = 0;
        releaseCounter = 0;
    }
    sei();
}

void releaseSwitch(uint8_t switchType) {
    releaseCounter = 0;
    while (bit_is_clear(PIND, switchType))
    {
        releaseCounter++;
        if (releaseCounter > 100)
        {
            break;
        }
        _delay_ms(20);
    }
    if (releaseCounter <= 100) {
        setState((switchType == UPPER_SWITCH_PIN ? STATE_STATIONARY_UP : STATE_STATIONARY_DOWN));
    }
    else {
        setState(STATE_ERROR);
    }
}

void tickFeeder() {
    if(state == STATE_STATIONARY_UP) {
        feederCounter++;
        if(feederCounter >= FEEDER_DELAY_COUNT) {
            PORTD |= (1 << MOTOR_FEEDER_PIN);
            _delay_ms(FEEDER_DURATION_MS);
            PORTD &= ~(1 << MOTOR_FEEDER_PIN);
            feederCounter = 0;
        }
    }
}



// Interrupts ----------------------------------------------------------
ISR(INT0_vect) {
    // upper switch made contact
    if (state == STATE_MOVING_UP) {
        setState(STATE_RELEASE_UPPER_SWITCH);
    }
}

ISR(INT1_vect)
{
    // lower switch made contact
    if (state == STATE_MOVING_DOWN){
        setState(STATE_RELEASE_LOWER_SWITCH);
    } 
}

ISR(WDT_vect) {
    if(inMovement == 1) {
        PORTB ^= (1 << LED_STATUS_PIN);
    } 
    
    if(watchDogTimerFlag == 0) {
        watchDogTimerFlag = 1;
    }
}


// main ------------------------------------------------------
int main() {
    // init outputs for LED
    DDRB |= (1 << LED_BATTERY_PIN) | (1 << LED_STATUS_PIN);

    // init outputs for motor
    DDRD |= (1 << MOTOR_DOOR_PIN_DOWN) | (1 << MOTOR_DOOR_PIN_UP) | (1 << MOTOR_FEEDER_PIN);
    DDRB |= (1 << MOTOR_DOOR_ENABLE_PIN);

    PORTD &= ~((1 << MOTOR_DOOR_PIN_DOWN) | (1 << MOTOR_DOOR_PIN_UP) | (1 << MOTOR_FEEDER_PIN));

    // init inputs for INT0/1 with pullup
    DDRD &= ~((1 << UPPER_SWITCH_PIN) | (1 << LOWER_SWITCH_PIN));
    PORTD |= (1 << UPPER_SWITCH_PIN) | (1 << LOWER_SWITCH_PIN);

    // set INT0/1 to trigger at any logical change
    EICRA |= (1 << ISC10) | (1 << ISC00);
    EIMSK |= (1 << INT1) | (1 << INT0);
    
    sei();

    // watchdog timer
    setWatchdogTimer(WDT_8000MS);

    // initially move up
    setState(STATE_MOVING_UP);

    
    // main loop ------------------------------------------------------
    while(1) {
        if(state == STATE_ERROR) {
            // blink status LED very fast
            PORTB ^= (1 << LED_STATUS_PIN);
            _delay_ms(100);
        } else {
            if (inMovement == 1) {
                if (state == STATE_RELEASE_UPPER_SWITCH) {
                    releaseSwitch(UPPER_SWITCH_PIN);
                }
                else if (state == STATE_RELEASE_LOWER_SWITCH) {
                    releaseSwitch(LOWER_SWITCH_PIN);
                }
                if(watchDogTimerFlag == 1) {
                    movementCounter++;
                    if(movementCounter > MAX_MOVEMENT_COUNTER) {
                        setState(STATE_ERROR);
                    }
                    watchDogTimerFlag = 0;
                }
            } else {
                if(watchDogTimerFlag == 1) {
                    switch(state) {
                        case STATE_STATIONARY_UP:
                            PORTB |= (1 << LED_STATUS_PIN);
                            _delay_ms(100);
                            PORTB &= ~(1 << LED_STATUS_PIN);
                            _delay_ms(150);
                            PORTB |= (1 << LED_STATUS_PIN);
                            _delay_ms(100);
                            PORTB &= ~(1 << LED_STATUS_PIN);
                            _delay_ms(150);
                            break;
                        case STATE_STATIONARY_DOWN:
                            PORTB |= (1 << LED_STATUS_PIN);
                            _delay_ms(100);
                            PORTB &= ~(1 << LED_STATUS_PIN);
                        break;
                        default: break;
                    }
                    tickFeeder();                    
                    watchDogTimerFlag = 0;
                }
            }
        }
    }

    return 0;
}