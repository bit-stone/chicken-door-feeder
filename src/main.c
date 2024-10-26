#include <avr/io.h>
#include <stddef.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/sleep.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>

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
#define ANALOG_CHANNEL_LIGHT 2

// amount of ticks after which idle state changes to moving
#define IDLE_TO_MOVING_TICK_COUNT 20

// if set, the transition will be done
// afterwards, flag has to be reset to zero
uint8_t state_transition_flag = FLAG_TRUE;

// if set, the idle mode will reset the light level to the
// next measure from analog input. value will be in EEPROM
uint8_t set_light_level_flag = FLAG_FALSE;

// current state
uint8_t current_state = STATE_MOVING_UP;

// delay return value
uint8_t delay_consumed = 0;

// amount of ticks used in current movement
uint8_t movement_tick_count = 0;
uint8_t release_tick_count = 0;

// amount of ticks for idle state change
uint8_t idle_tick_count = 0;

// used to store the currently measured light level
uint16_t light_level = 1024;

// light_level to check against. Read from EEPROM.
// to prevent unneccesary state changes, it has to be lower for moving down
// than for moving up (about 5% or something maybe?)
uint16_t target_light_level = 0;

// IMPORTANT: This is only used when the EEPROM value is read as 0!
// Otherwise the eeprom value will be used.
#define FALLBACK_TARGET_LIGHT_LEVEL 500

// this will be multiplied with the current light level while in IDLE_DOWN
// to prevent the door from opening up again. Meaning:
// To go up, it has to be a bit brigther compared to going down.
// Also: To go down, it has to be a bit darker compared to going up.
// This factor should be < 1.0
#define LIGHT_LEVEL_DOWN_SUBTRACT_VALUE 15

#define EEPROM_LIGHT_LEVEL_ADDRESS 4

// counter
uint16_t k = 0;
uint16_t l = 0;
uint16_t m = 0;

uint16_t while_counter = 0;

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

void write_new_target_light_level(uint16_t new_level)
{
  // if the button was pressed set new value for brightness
  // wanted behaviour: If pressed while in STATE_IDLE_UP, the door should go into
  // closing behavior on the next blink

  eeprom_write_word((uint16_t *)EEPROM_LIGHT_LEVEL_ADDRESS, new_level);
  target_light_level = new_level;

  // might be useful to output the new light level somehow...
  led_debug_off();
  delay_and_check_flag(100);
  for (m = 11; m >= 1; m--)
  {
    led_debug_on();
    delay_consumed = delay_and_check_flag(200);

    led_debug_off();
    delay_consumed = delay_and_check_flag(200);

    if (((new_level >> (m - 1)) & 0b1 )== 1)
    {
      led_debug_on();
    }
    else
    {
      led_debug_off();
    }
    delay_consumed = delay_and_check_flag(200);

    led_debug_off();
    delay_consumed = delay_and_check_flag(1000);

    if(delay_consumed == FLAG_FALSE) {
      return;
    }
  }
}

// check if end stops are shown as "LOW" and return 1
// if not, return 0
uint8_t check_end_stops()
{
  // Both endstops have to be low when in idle mode
  // since they are normally closed to GND.
  if (
      !bit_is_clear(LIMIT_SWITCH_PIN, LIMIT_SWITCH_UP_PIN) ||
      !bit_is_clear(LIMIT_SWITCH_PIN, LIMIT_SWITCH_DOWN_PIN))
  {
    return FLAG_FALSE;
  }

  return FLAG_TRUE;
}

// ~~~~~~~~~~~~~~~~~~ ISR ~~~~~~~~~~~~~~~~~~
// INT0 = PD2 = green = upper switch was depressed (low -> high transition)
ISR(INT0_vect)
{
  wdt_reset();
  // the motor was moving up and the upper limit switch has been pressed
  if (current_state == STATE_MOVING_UP && bit_is_set(LIMIT_SWITCH_PIN, LIMIT_SWITCH_UP_PIN))
  {
    request_transition(STATE_RELEASING_UP);
  }
  // the motor was moving down (to release) and the upper limit switch has been released
  else if (current_state == STATE_RELEASING_UP && bit_is_clear(LIMIT_SWITCH_PIN, LIMIT_SWITCH_UP_PIN))
  {
    request_transition(STATE_IDLE_UP);
  }
}

ISR(INT1_vect)
{
  wdt_reset();
  // the motor was moving down and the lower limit switch has been pressed
  if (current_state == STATE_MOVING_DOWN && bit_is_set(LIMIT_SWITCH_PIN, LIMIT_SWITCH_DOWN_PIN))
  {
    request_transition(STATE_RELEASING_DOWN);
  }
  // the motor was moving up (to release) and the lower limit switch has been released
  else if (current_state == STATE_RELEASING_DOWN && bit_is_clear(LIMIT_SWITCH_PIN, LIMIT_SWITCH_DOWN_PIN))
  {
    request_transition(STATE_IDLE_DOWN);
  }
}

// only used to wake up. Might be useful later.
ISR(WDT_vect)
{
  ;
}

ISR(PCINT0_vect)
{
  wdt_reset();
  // figure out which pin caused the interrupt and if any was pressed or just released
  if (bit_is_clear(PINB, PB0))
  {
    if ((current_state == STATE_IDLE_UP || current_state == STATE_IDLE_DOWN) && set_light_level_flag == FLAG_FALSE)
    {
      set_light_level_flag = FLAG_TRUE;
    }
  }
  else if (bit_is_clear(PINB, PB1))
  {
    if (current_state == STATE_IDLE_UP)
    {
      request_transition(STATE_MOVING_DOWN);
    }
  }
  else if (bit_is_clear(PINB, PB2))
  {
    if (current_state == STATE_IDLE_DOWN)
    {
      request_transition(STATE_MOVING_UP);
    }
  }
}

// ##########################################################################
int main()
{
  // startup stuff
  led_init();
  led_debug_init();
  motor_init();

  // read target light level from eeprom.
  // if 0, set the default value
  // first wait for eeprom to be ready
  while_counter = 0;
  while (!eeprom_is_ready() && while_counter < 10000)
  {
    while_counter++;
  }

  if (while_counter < 10000)
  {
    target_light_level = eeprom_read_word((uint16_t *)EEPROM_LIGHT_LEVEL_ADDRESS);
  }

  if (target_light_level == 0 || target_light_level == 0xffff)
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

  // setting up Pin change interrupts on PB0 - PB2
  // these values are not defined, as the PCINT has to be on this port and other pins are not available
  DDRB &= ~((1 << PB0) | (1 << PB1) | (1 << PB2));
  PORTB |= (1 << PB0) | (1 << PB1) | (1 << PB2); // pullups active

  PCICR |= (1 << PCIE0);
  PCMSK0 |= (1 << PCINT2) | (1 << PCINT1) | (1 << PCINT0);

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

  // set sleep mode
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();

  // configure watchdog to interrupt about 8 seconds
  MCUSR &= ~(1 << WDRF);
  WDTCSR |= (1 << WDCE) | (1 << WDE);
  WDTCSR = 0b01100001;

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
  switch_state_start:
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

      led_on();
      delay_consumed = delay_and_check_flag(50);
      if (delay_consumed == DELAY_ABORTED)
      {
        break;
      }

      led_off();
      delay_consumed = delay_and_check_flag(750);
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

        if (check_end_stops() == FLAG_FALSE)
        {
          request_transition(STATE_ERROR);
          break;
        }

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

        if (check_end_stops() == FLAG_FALSE)
        {
          request_transition(STATE_ERROR);
          break;
        }

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

        // reset idle ticks
        idle_tick_count = 0;

        // show idle transition led signal
        for (k = 0; k < 3; k++)
        {
          led_off();
          delay_consumed = delay_and_check_flag(50);
          if (delay_consumed == DELAY_ABORTED)
          {
            goto switch_state_start;
          }

          led_on();
          delay_consumed = delay_and_check_flag(50);
          if (delay_consumed == DELAY_ABORTED)
          {
            goto switch_state_start;
          }
        }

        led_off();
      }

      // show idle state animation
      led_off();
      delay_consumed = delay_and_check_flag(50);
      if (delay_consumed == DELAY_ABORTED)
      {
        break;
      }

      for (k = 0; k < 2; k++)
      {
        led_on();
        delay_consumed = delay_and_check_flag(100);
        if (delay_consumed == DELAY_ABORTED)
        {
          goto switch_state_start;
        }

        led_off();
        delay_consumed = delay_and_check_flag(50);
        if (delay_consumed == DELAY_ABORTED)
        {
          goto switch_state_start;
        }
      }

      // measure brightness
      light_level = analog_measure_value(ANALOG_CHANNEL_LIGHT);

      // used to debug analog values
      // led_debug_on();
      // for(uint16_t m = 0; m < target_light_level; m++) {
      //   _delay_ms(1);
      // }
      // led_debug_off();

      if (set_light_level_flag == FLAG_TRUE)
      {
        write_new_target_light_level(light_level);
        set_light_level_flag = FLAG_FALSE;
      }

      if (light_level < target_light_level)
      {
        idle_tick_count++;
        led_on();
      }
      else
      {
        idle_tick_count = 0;
        led_off();
      }

      if (idle_tick_count > IDLE_TO_MOVING_TICK_COUNT)
      {
        request_transition(STATE_MOVING_DOWN);
        break;
      }

      // _delay_ms(8000);
      // does not work for some reason?
      sleep_cpu();

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
        // reset idle ticks
        idle_tick_count = 0;

        // show idle transition led signal
        for (k = 0; k < 2; k++)
        {
          led_off();
          delay_consumed = delay_and_check_flag(50);
          if (delay_consumed == DELAY_ABORTED)
          {
            goto switch_state_start;
          }

          led_on();
          delay_consumed = delay_and_check_flag(200);
          if (delay_consumed == DELAY_ABORTED)
          {
            goto switch_state_start;
          }
        }

        led_off();
      }

      // show idle state animation
      led_off();
      delay_consumed = delay_and_check_flag(50);
      if (delay_consumed == DELAY_ABORTED)
      {
        break;
      }

      led_on();
      delay_consumed = delay_and_check_flag(70);
      if (delay_consumed == DELAY_ABORTED)
      {
        break;
      }

      led_off();
      delay_consumed = delay_and_check_flag(50);
      if (delay_consumed == DELAY_ABORTED)
      {
        break;
      }

      // measure brightness
      light_level = analog_measure_value(ANALOG_CHANNEL_LIGHT);

      if (set_light_level_flag == FLAG_TRUE)
      {
        write_new_target_light_level(light_level);
        set_light_level_flag = FLAG_FALSE;
      }

      if (light_level > (target_light_level + LIGHT_LEVEL_DOWN_SUBTRACT_VALUE))
      {
        idle_tick_count++;
        led_on();
      }
      else
      {
        idle_tick_count = 0;
        led_off();
      }

      if (idle_tick_count > IDLE_TO_MOVING_TICK_COUNT)
      {
        request_transition(STATE_MOVING_UP);
        break;
      }

      // _delay_ms(8000);
      sleep_cpu();

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