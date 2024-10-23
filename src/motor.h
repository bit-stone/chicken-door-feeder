#include <avr/io.h>
#include <util/delay.h>

// port and ddr for motor (all need to be on the same port)
#define MOTOR_PORT PORTD
#define MOTOR_DDR DDRD

// which pin to use for enabling the output motor 
// -> h-bridge will be powered
#define MOTOR_ENABLE_PIN PD4

// which pins need to be set to high to make the motor move in that direction
// IMPORTANT: Never set both high, as that will short out the h-bridge
#define MOTOR_UP_PIN PD7
#define MOTOR_DOWN_PIN PD6

void motor_init();

void motor_stop();
void motor_move_up();
void motor_move_down();

void _motor_enable();
void _motor_disable();