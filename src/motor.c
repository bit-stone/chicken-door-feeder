#include <motor.h>

void motor_init()
{
    // set pins as output
    MOTOR_DDR |= (1 << MOTOR_ENABLE_PIN) | (1 << MOTOR_UP_PIN) | (1 << MOTOR_DOWN_PIN);
    
    // set all output low
    MOTOR_PORT &= ~(1 << MOTOR_ENABLE_PIN);
    MOTOR_PORT &= ~(1 << MOTOR_UP_PIN);
    MOTOR_PORT &= ~(1 << MOTOR_DOWN_PIN);
}

void motor_stop()
{
    // disable all movement
    MOTOR_PORT &= ~(1 << MOTOR_UP_PIN);
    MOTOR_PORT &= ~(1 << MOTOR_DOWN_PIN);

    // disable motor itself
    _motor_disable();
}

void motor_move_up()
{
    _motor_enable();
    // to be sure, always reset the other pin before setting a new one
    // to avoid a short

    MOTOR_PORT &= ~(1 << MOTOR_DOWN_PIN);
    _delay_ms(50);
    MOTOR_PORT |= (1 << MOTOR_UP_PIN);
}

void motor_move_down()
{
    _motor_enable();
    // to be sure, always reset the other pin before setting a new one
    // to avoid a short

    MOTOR_PORT &= ~(1 << MOTOR_UP_PIN);
    _delay_ms(50);
    MOTOR_PORT |= (1 << MOTOR_DOWN_PIN);
}

void _motor_enable()
{
    MOTOR_PORT |= (1 << MOTOR_ENABLE_PIN);
}

void _motor_disable()
{
    MOTOR_PORT &= ~(1 << MOTOR_ENABLE_PIN);
}
