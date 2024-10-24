#include "analog.h"
#include "led.h"

uint16_t analog_input = 0;
uint16_t analog_input_count = 0;
uint16_t analog_value = 0;

uint16_t _while_counter = 0;

uint16_t analog_measure_value(uint8_t analog_channel)
{
    if(analog_channel > 8) {
        analog_channel = 8;
    }

    // set internal 1.1v reference
    ADMUX |= (1 << REFS1) | (1 << REFS0);

    // set analog channel
    // ADMUX = (ADMUX & 0xF0) | (analog_channel & 0x0F);
    ADMUX |= (1 << MUX1);

    // set prescaler 16 -> 1 MHz / 16 = 62.5kHz
    ADCSRA |= (1 << ADPS2);

    // enable ADC (now consumes power)
    ADCSRA |= (1 << ADEN);

    // stabilizing time
    _delay_us(ANALOG_MEASUREMENT_FREERUN_US);

    analog_input_count = 0;
    analog_input = 0;
    analog_value = 0;

    while(analog_input_count <= ANALOG_MEASUREMENT_COUNT) {
        ADCSRA |= (1 << ADSC);
        _while_counter = 0;
        while (bit_is_set(ADCSRA, ADSC) && _while_counter < 10000)
        {
            _while_counter++;
        }
        analog_input = ADCW;
        analog_value += analog_input;
        analog_input_count++;
    }

    analog_value = analog_value / analog_input_count;

    // disable ADC (save power)
    ADCSRA &= ~(1 << ADEN);

    // return 1024;
    return analog_value;
}