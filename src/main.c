
#include <avr/io.h>
#include <stddef.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/sleep.h>

// MAX Settings
#define MAX_MOVEMENT_COUNTER 70 // watchdog timer counts twice per second

// must be lower then 16 bit for all 1024 values (65535 / 1024 = 63,9)
// MAX value is 63
#define ANALOG_MEASUREMENT_FREERUN_US 10
#define ANALOG_MEASUREMENT_COUNT 10

#define ADMUX_VALUE 0x00;

// light change analog value
// brighter -> higher voltage -> higher value
// #define LIGHT_CHANGE_ADV_VALUE 585
#define LIGHT_CHANGE_ADV_VALUE 240

#define STATE_CHANGE_COUNT 10

// Definitions ------------------------------------------------------
#define MOTOR_DOOR_PIN_UP PD7
#define MOTOR_DOOR_PIN_DOWN PD6

// #define MOTOR_FEEDER_PIN PD5
#define MOTOR_FEEDER_PIN PB7

// #define MOTOR_DOOR_ENABLE_PIN PB2
#define MOTOR_DOOR_ENABLE_PIN PD5

#define UPPER_SWITCH_PIN PD2
#define LOWER_SWITCH_PIN PD3

#define LED_BATTERY_PIN PB1
#define LED_STATUS_PIN PB0

#define ADC_CHANNEL_LIGHT 1

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
#define WDT_32MS 65   //    0        1         0         0        0        0          0          1
#define WDT_64MS 66   //    0        1         0         0        0        0          1          0
#define WDT_125MS 67  //    0        1         0         0        0        0          1          1
#define WDT_250MS 68  //    0        1         0         0        0        0          0          0
#define WDT_500MS 69  //    0        1         0         0        0        1          0          0
#define WDT_1000MS 70 //    0        1         0         0        0        1          0          1
#define WDT_2000MS 71 //    0        1         0         0        0        1          1          1
#define WDT_4000MS 96 //    0        1         1         0        0        0          0          0
#define WDT_8000MS 97

volatile uint16_t analogInput = 0;
volatile uint8_t analogCount = 0;
volatile uint16_t analogValue = 0;

volatile uint8_t state = STATE_STATIONARY_UP;
volatile uint8_t inMovement = 0;

uint8_t releaseCounter = 0;
uint16_t movementCounter = 0;
uint16_t feederCounter = 0;

uint8_t watchDogTimerFlag = 0;

uint8_t stateChangeCounter = 0;

// Global methods ------------------------------------------------------

// enable door motor h-bridge
void enableDoorMotor()
{
  PORTD |= (1 << MOTOR_DOOR_ENABLE_PIN);
}

// disable door motor h-brigde
void disableDoorMotor()
{
  PORTD &= ~(1 << MOTOR_DOOR_ENABLE_PIN);
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
void setDoorMotor(uint8_t direction)
{
  switch (direction)
  {
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

void setState(uint8_t newState)
{
  cli();
  if (newState != state)
  {
    switch (newState)
    {
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

void releaseSwitch(uint8_t switchType)
{
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
  if (releaseCounter <= 100)
  {
    setState((switchType == UPPER_SWITCH_PIN ? STATE_STATIONARY_UP : STATE_STATIONARY_DOWN));
  }
  else
  {
    setState(STATE_ERROR);
  }
}

void measureAnalogValue(uint8_t analogChannel)
{
  // check max value of channel
  // to avoid changing of other settings
  if (analogChannel > 15)
  {
    analogChannel = 15;
  }

  // set analog channel
  ADCSRA |= (1 << ADEN);
  ADCSRA |= (1 << ADPS2);
  ADMUX |= (ADMUX & 0xF0) | (analogChannel & 0x0F);

  // stabilizing time
  _delay_us(ANALOG_MEASUREMENT_FREERUN_US);

  // reset values
  analogCount = 0;
  analogInput = 0;

  // read current values
  while (analogCount <= ANALOG_MEASUREMENT_COUNT)
  {
    ADCSRA |= (1 << ADSC);
    while (bit_is_set(ADCSRA, ADSC))
    {
      ;
    }
    analogInput = ADCW;
    analogValue += analogInput;
    analogCount++;
  }

  // calculate average
  analogValue = analogValue / analogCount;
}

// Interrupts ----------------------------------------------------------
ISR(INT0_vect)
{
  // upper switch made contact
  if (state == STATE_MOVING_UP)
  {
    setState(STATE_RELEASE_UPPER_SWITCH);
  }

  if (state == STATE_STATIONARY_UP)
  {
    setState(STATE_MOVING_DOWN);
  }
}

ISR(INT1_vect)
{
  // lower switch made contact
  if (state == STATE_MOVING_DOWN)
  {
    setState(STATE_RELEASE_LOWER_SWITCH);
  }

  // removed -> to dangerous if animals hit it by accident
  // same could be achieved by simple reset
  // if(state == STATE_STATIONARY_DOWN) {
  //   setState(STATE_MOVING_UP);
  // }
}

ISR(WDT_vect)
{
  if (inMovement == 1)
  {
    PORTB ^= (1 << LED_STATUS_PIN);
  }

  if (watchDogTimerFlag == 0)
  {
    watchDogTimerFlag = 1;
  }
}

// main ------------------------------------------------------
int main()
{
  // init outputs for LED
  DDRB |= (1 << LED_BATTERY_PIN) | (1 << LED_STATUS_PIN);

  // init outputs for motor
  DDRB |= (1 << MOTOR_FEEDER_PIN);
  DDRD |= (1 << MOTOR_DOOR_PIN_DOWN) | (1 << MOTOR_DOOR_PIN_UP);
  DDRD |= (1 << MOTOR_DOOR_ENABLE_PIN);

  PORTB &= ~(1 << MOTOR_FEEDER_PIN);
  PORTD &= ~((1 << MOTOR_DOOR_PIN_DOWN) | (1 << MOTOR_DOOR_PIN_UP));

  // init inputs for INT0/1 with pullup
  DDRD &= ~((1 << UPPER_SWITCH_PIN) | (1 << LOWER_SWITCH_PIN));
  PORTD |= (1 << UPPER_SWITCH_PIN) | (1 << LOWER_SWITCH_PIN);

  // set INT0/1 to trigger at any logical change
  EICRA |= (1 << ISC10) | (1 << ISC00);
  EIMSK |= (1 << INT1) | (1 << INT0);

  ADMUX = ADMUX_VALUE;

  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sei();

  // watchdog timer
  setWatchdogTimer(WDT_8000MS);

  // check if upper switch is pressed
  // otherwise move up
  if (bit_is_clear(PIND, PD2))
  {
    setState(STATE_RELEASE_UPPER_SWITCH);
  }
  else
  {
    setState(STATE_MOVING_UP);
  }

  // main loop ------------------------------------------------------
  while (1)
  {
    if (state == STATE_ERROR)
    {
      // blink status LED very fast
      PORTB ^= (1 << LED_STATUS_PIN);
      _delay_ms(100);
    }
    else
    {
      if (inMovement == 1)
      {
        if (state == STATE_RELEASE_UPPER_SWITCH)
        {
          releaseSwitch(UPPER_SWITCH_PIN);
        }
        else if (state == STATE_RELEASE_LOWER_SWITCH)
        {
          releaseSwitch(LOWER_SWITCH_PIN);
        }
        if (watchDogTimerFlag == 1)
        {
          movementCounter++;
          if (movementCounter > MAX_MOVEMENT_COUNTER)
          {
            setState(STATE_ERROR);
          }
          watchDogTimerFlag = 0;
        }
      }
      else
      {
        if (watchDogTimerFlag == 1)
        {
          cli();
          switch (state)
          {
          case STATE_STATIONARY_UP:
            PORTB &= ~(1 << LED_STATUS_PIN);
            _delay_ms(150);
            PORTB |= (1 << LED_STATUS_PIN);
            _delay_ms(100);
            PORTB &= ~(1 << LED_STATUS_PIN);
            _delay_ms(150);
            PORTB |= (1 << LED_STATUS_PIN);
            _delay_ms(100);
            PORTB &= ~(1 << LED_STATUS_PIN);
            _delay_ms(150);

            // measure brightness
            measureAnalogValue(ADC_CHANNEL_LIGHT);
            if (analogValue <= LIGHT_CHANGE_ADV_VALUE)
            {
              stateChangeCounter++;
              PORTB |= (1 << LED_STATUS_PIN);
            }
            else
            {
              stateChangeCounter = 0;
              PORTB &= ~(1 << LED_STATUS_PIN);
            }

            if (stateChangeCounter > STATE_CHANGE_COUNT)
            {
              setState(STATE_MOVING_DOWN);
            }
            break;
          case STATE_STATIONARY_DOWN:
            PORTB &= ~(1 << LED_STATUS_PIN);
            _delay_ms(150);
            PORTB |= (1 << LED_STATUS_PIN);
            _delay_ms(100);
            PORTB &= ~(1 << LED_STATUS_PIN);
            _delay_ms(150);

            measureAnalogValue(ADC_CHANNEL_LIGHT);
            if (analogValue >= LIGHT_CHANGE_ADV_VALUE)
            {
              stateChangeCounter++;
              PORTB |= (1 << LED_STATUS_PIN);
            }
            else
            {
              stateChangeCounter = 0;
              PORTB &= ~(1 << LED_STATUS_PIN);
            }

            if (stateChangeCounter > STATE_CHANGE_COUNT)
            {
              setState(STATE_MOVING_UP);
            }
            break;
          default:
            break;
          }
          watchDogTimerFlag = 0;
          ADCSRA &= ~(1 << ADEN);
          sei();
          sleep_mode();
        }
      }
    }
  }

  return 0;
}
