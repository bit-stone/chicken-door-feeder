#include <avr/io.h>
#include <stddef.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/sleep.h>

#include "led.h"
#include "motor.h"
#include "analog.h"

// the different states of this component
// sleep() is only used within the IDLE_XX states
// the order matters, as apart from STATE_ERROR every next state can
// only be transitions to from the previous state.
#define STATE_ERROR 0
#define STATE_MOVING_UP 1
#define STATE_RELEASING_UP 2
#define STATE_IDLE_UP 3
#define STATE_MOVING_DOWN 4
#define STATE_RELEASING_DOWN 5
#define STATE_IDLE_DOWN 6

// where the NC endstops are connected
// need to be on INT0/1
#define LIMIT_SWITCH_PORT PORTD
#define LIMIT_SWITCH_DDR DDRD
#define LIMIT_SWITCH_PIN PIND
#define LIMIT_SWITCH_UP_PIN PD2
#define LIMIT_SWITCH_DOWN_PIN PD3

// settings to prevent human read errors :)
#define FLAG_TRUE 1
#define FLAG_FALSE 0

// return values for when a delay was completed or aborted
#define DELAY_COMPLETED 1
#define DELAY_ABORTED 0

// the minimum delay between flag checks for state transitions
// this delay will be the maximum time between requesting
// a transition and the code actually doing one
#define DELAY_TICK_MS 5

// the amount of ticks until a timeout is reached for MOVING_UP/MOVING_DOWN
// One tick thereby is one complete while-round and mostly depends on the amount
// of _delay_ms() used. For 2x500ms this would be around 60 seconds.
#define MOVING_STATE_TICK_MAX_COUNT 60

// for relase, the tick is much shorter (100ms)
#define RELEASE_STATE_TICK_MAX_COUNT 20

// channel on which the light level is measured
#define ANALOG_CHANNEL_LIGHT 0

// if set, the transition will be done
// afterwards, flag has to be reset to zero
uint8_t state_transition_flag = FLAG_TRUE;

// if set, the idle mode will reset the light level to the
// next measure from analog input. value will be in EEPROM
uint8_t set_light_level_flag = 0;

// current state
uint8_t current_state = STATE_MOVING_UP;

// delay return value
uint8_t delay_consumed = 0;

// amount of ticks used in current movement
uint8_t movement_tick_count = 0;
uint8_t release_tick_count = 0;

// used to store the currently measured light level
uint16_t light_level = 0;

// light_level to check against. Read from EEPROM.
// to prevent unneccesary state changes, it has to be lower for moving down
// than for moving up (about 5% or something maybe?)
uint16_t target_light_level = 0;

// IMPORTANT: This is only used when the EEPROM value is read as 0!
// Otherwise the eeprom value will be used.
#define FALLBACK_TARGET_LIGHT_LEVEL 500

// counter
uint8_t k = 0;
uint8_t l = 0;

/**
 * Request a state transition to a new state
 */
void request_transition(uint8_t target_state)
{
  if (target_state == current_state)
  {
    return;
  }

  if (target_state == STATE_ERROR)
  {
    // switching to error state is always allowed
    state_transition_flag = FLAG_TRUE;
    current_state = STATE_ERROR;
    return;
  }

  if (target_state != STATE_MOVING_UP)
  {
    if (target_state == current_state + 1)
    {
      state_transition_flag = FLAG_TRUE;
      current_state = target_state;
      return;
    }
  }
  else
  {
    // have to handle STATE_IDLE_DOWN because of wrapping around
    if (current_state == STATE_IDLE_DOWN)
    {
      state_transition_flag = FLAG_TRUE;
      current_state = target_state;
      return;
    }
  }
}

// delay for a certain amount. If the flag is set before
// time runs out, return 0 to show, that a break needs to happen
uint8_t delay_and_check_flag(uint16_t amount)
{
  for (l = 0; l < (amount / DELAY_TICK_MS); l++)
  {
    _delay_ms(DELAY_TICK_MS);
    if (state_transition_flag == FLAG_TRUE)
    {
      return DELAY_ABORTED;
    }
  }

  return DELAY_COMPLETED;
}

// ~~~~~~~~~~~~~~~~~~ ISR ~~~~~~~~~~~~~~~~~~
// INT0 = PD2 = green = upper switch was depressed (low -> high transition)
ISR(INT0_vect)
{
  // the motor was moving up and the upper limit switch has been pressed
  if (current_state == STATE_MOVING_UP && bit_is_set(LIMIT_SWITCH_PIN, LIMIT_SWITCH_UP_PIN))
  {
    request_transition(STATE_RELEASING_UP);
    return;
  }

  // the motor was moving down (to release) and the upper limit switch has been released
  if (current_state == STATE_RELEASING_UP && bit_is_clear(LIMIT_SWITCH_PIN, LIMIT_SWITCH_UP_PIN))
  {
    request_transition(STATE_IDLE_UP);
    return;
  }
}

ISR(INT1_vect)
{
  // the motor was moving down and the lower limit switch has been pressed
  if (current_state == STATE_MOVING_DOWN && bit_is_set(LIMIT_SWITCH_PIN, LIMIT_SWITCH_DOWN_PIN))
  {
    request_transition(STATE_RELEASING_DOWN);
    return;
  }

  // the motor was moving up (to release) and the lower limit switch has been released
  if (current_state == STATE_RELEASING_DOWN && bit_is_clear(LIMIT_SWITCH_PIN, LIMIT_SWITCH_DOWN_PIN))
  {
    request_transition(STATE_IDLE_DOWN);
    return;
  }
}

// ##########################################################################
int main()
{
  // startup stuff
  led_init();
  motor_init();

  // read target light level from eeprom.
  // if 0, set the default value

  if (target_light_level == 0)
  {
    target_light_level = FALLBACK_TARGET_LIGHT_LEVEL;
  }

  // setting both interrupts to trigger on low-high-transition
  // and have the pullup enabled
  LIMIT_SWITCH_DDR &= ~((1 << LIMIT_SWITCH_UP_PIN) | 1 << LIMIT_SWITCH_DOWN_PIN);
  LIMIT_SWITCH_PORT |= (1 << LIMIT_SWITCH_UP_PIN) | (1 << LIMIT_SWITCH_DOWN_PIN);

  // setting both interrupts to trigger on any logic level change
  // (low->high for endstop, high->low for release needed)
  EICRA |= (1 << ISC10) | (1 << ISC00);

  // enable both interrupts
  EIMSK |= (1 << INT1) | (1 << INT0);

  // show led startup animation
  led_on();
  _delay_ms(50);
  led_off();
  _delay_ms(50);

  led_on();
  _delay_ms(50);
  led_off();
  _delay_ms(50);

  led_on();
  _delay_ms(50);
  led_off();
  _delay_ms(500);

  // enable interrupts globally
  sei();

  // if the upper end switch is pressed already,
  // prevent damage by setting the release_upper state
  if (bit_is_set(LIMIT_SWITCH_PIN, LIMIT_SWITCH_UP_PIN))
  {
    current_state = STATE_RELEASING_UP;
    state_transition_flag = FLAG_TRUE;
  }

  // loop
  while (1)
  {
    switch (current_state)
    {
    // ERROR STATE
    // ~~~~~~~~~~~~~~~~~~~~~~ STATE_ERROR ~~~~~~~~~~~~~~~~~~~~
    case STATE_ERROR:
      if (state_transition_flag == FLAG_TRUE)
      {
        state_transition_flag = FLAG_FALSE;
        // disable all interrupts
        cli();
        // stop all movement
        motor_stop();
        // reset the LED
        led_off();
      }

      // blink kind of fast
      led_on();
      delay_consumed = delay_and_check_flag(50);
      if (delay_consumed == DELAY_ABORTED)
      {
        break;
      }

      led_off();
      delay_consumed = delay_and_check_flag(250);
      if (delay_consumed == DELAY_ABORTED)
      {
        break;
      }

      break;

    // MOVING UP AND DOWN
    // ~~~~~~~~~~~~~~~~~~~~~~ STATE_MOVING_UP ~~~~~~~~~~~~~~~~~~~~
    case STATE_MOVING_UP:
      if (state_transition_flag == FLAG_TRUE)
      {
        state_transition_flag = FLAG_FALSE;
        // reset LED
        led_off();
        // start moving up
        motor_move_up();
        // reset move tick count
        movement_tick_count = 0;
      }

      led_on();
      delay_consumed = delay_and_check_flag(500);
      if (delay_consumed == DELAY_ABORTED)
      {
        break;
      }

      led_off();
      delay_consumed = delay_and_check_flag(500);
      if (delay_consumed == DELAY_ABORTED)
      {
        break;
      }

      movement_tick_count++;

      if (movement_tick_count > MOVING_STATE_TICK_MAX_COUNT)
      {
        request_transition(STATE_ERROR);
        break;
      }

      break;
    // ~~~~~~~~~~~~~~~~~~~~~~ STATE_MOVING_DOWN ~~~~~~~~~~~~~~~~~~~~
    case STATE_MOVING_DOWN:
      if (state_transition_flag == FLAG_TRUE)
      {
        state_transition_flag = FLAG_FALSE;
        // reset LED
        led_off();
        // start moving down
        motor_move_down();
        // reset move tick count
        movement_tick_count = 0;
      }

      led_on();
      delay_consumed = delay_and_check_flag(500);
      if (delay_consumed == DELAY_ABORTED)
      {
        break;
      }

      led_off();
      delay_consumed = delay_and_check_flag(500);
      if (delay_consumed == DELAY_ABORTED)
      {
        break;
      }

      movement_tick_count++;

      if (movement_tick_count > MOVING_STATE_TICK_MAX_COUNT)
      {
        request_transition(STATE_ERROR);
        break;
      }

      break;
    // RELEASING UP AND DOWN
    // ~~~~~~~~~~~~~~~~~~~~~~ STATE_RELEASING_UP ~~~~~~~~~~~~~~~~~~~~
    case STATE_RELEASING_UP:
      if (state_transition_flag == FLAG_TRUE)
      {
        state_transition_flag = FLAG_FALSE;
        // reset LED (always on in release state)
        led_on();
        // start moving down (to release upper switch)
        motor_move_down();
        // reset release ticks
        release_tick_count = 0;
      }

      delay_consumed = delay_and_check_flag(100);
      if (delay_consumed == DELAY_ABORTED)
      {
        break;
      }

      release_tick_count++;
      if (release_tick_count > RELEASE_STATE_TICK_MAX_COUNT)
      {
        request_transition(STATE_ERROR);
        break;
      }

      break;
    // ~~~~~~~~~~~~~~~~~~~~~~ STATE_RELEASING_DOWN ~~~~~~~~~~~~~~~~~~~~
    case STATE_RELEASING_DOWN:
      if (state_transition_flag == FLAG_TRUE)
      {
        state_transition_flag = FLAG_FALSE;
        // reset LED (always on in release state)
        led_on();
        // start moving up
        motor_move_up();
        // reset release ticks
        release_tick_count = 0;
      }

      delay_consumed = delay_and_check_flag(100);
      if (delay_consumed == DELAY_ABORTED)
      {
        break;
      }

      release_tick_count++;
      if (release_tick_count > RELEASE_STATE_TICK_MAX_COUNT)
      {
        request_transition(STATE_ERROR);
        break;
      }

      break;

    // IDLE STATES
    // ~~~~~~~~~~~~~~~~~~~~~~ STATE_IDLE_UP ~~~~~~~~~~~~~~~~~~~~
    case STATE_IDLE_UP:
      if (state_transition_flag == FLAG_TRUE)
      {
        state_transition_flag = FLAG_FALSE;

        // reset LED
        led_off();
        // stop all movement
        motor_stop();

        // show idle transition led signal
        for (k = 0; k < 5; k++)
        {
          led_off();
          delay_consumed = delay_and_check_flag(100);
          if (delay_consumed == DELAY_ABORTED)
          {
            break;
          }

          led_on();
          delay_consumed = delay_and_check_flag(400);
          if (delay_consumed == DELAY_ABORTED)
          {
            break;
          }

          led_off();
        }
      }

      led_on();
      // measure brightness
      analog_measure_value(ANALOG_CHANNEL_LIGHT, &light_level);

      if (set_light_level_flag == FLAG_TRUE)
      {
        // if the button was pressed set new value for brightness
        // wanted behaviour: If pressed while in STATE_IDLE_UP, the door should go into
        // closing behavior on the next blink
      }

      led_off();

      break;
    // ~~~~~~~~~~~~~~~~~~~~~~ STATE_IDLE_DOWN ~~~~~~~~~~~~~~~~~~~~
    case STATE_IDLE_DOWN:
      if (state_transition_flag == FLAG_TRUE)
      {
        state_transition_flag = FLAG_FALSE;
        // reset LED
        led_off();
        // stop all movement
        motor_stop();

        // show idle transition led signal
        for (k = 0; k < 2; k++)
        {
          led_off();
          delay_consumed = delay_and_check_flag(100);
          if (delay_consumed == DELAY_ABORTED)
          {
            break;
          }

          led_on();
          delay_consumed = delay_and_check_flag(400);
          if (delay_consumed == DELAY_ABORTED)
          {
            break;
          }
        }

        led_off();
      }

      led_on();
      // measure brightness (again?)
      if (set_light_level_flag == FLAG_TRUE)
      {
        // if the button was pressed set new value for brightness
        // wanted behaviour: If pressed while in STATE_IDLE_UP, the door should go into
        // closing behavior on the next blink
      }
      led_off();

      break;

    // this should never happen :)
    default:
      request_transition(STATE_ERROR);
      break;
    }
  }

  return 0;
}