#include "analog.h"

uint16_t analog_input = 0;
uint8_t analog_input_count = 0;
uint16_t analog_value = 0;

void analog_measure_value(uint8_t analog_channel, uint16_t *value_out)
{
    if(analog_channel > 8) {
        analog_channel = 8;
    }

    // set internal 1.1v reference
    ADMUX |= (1 << REFS1) | (1 << REFS0);

    // set analog channel
    ADMUX |= (ADMUX & 0xF0) | (analog_channel & 0x0F);

    // set prescaler 8 -> 1 MHz / 8 = 125kHz
    ADCSRA |= (1 << ADPS1) | (1 << ADPS0);

    // enable ADC (now consumes power)
    ADCSRA |= (1 << ADEN);

    // stabilizing time
    _delay_us(ANALOG_MEASUREMENT_FREERUN_US);

    analog_input_count = 0;
    analog_input = 0;

    while(analog_input_count <= ANALOG_MEASUREMENT_COUNT) {
        ADCSRA |= (1 << ADSC);
        while (bit_is_set(ADCSRA, ADSC))
        {
            ;
        }
        analog_input = ADCW;
        analog_value += analog_input;
        analog_input_count++;
    }

    analog_value = analog_input / analog_input_count;

    *value_out = analog_value;

    // disable ADC (save power)
    ADCSRA &= ~(1 << ADEN);
}