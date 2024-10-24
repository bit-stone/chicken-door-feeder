#include <avr/io.h>
#include <util/delay.h>

// time to wait until starting analog measurement
#define ANALOG_MEASUREMENT_FREERUN_US 200

// number of runs per measurement (which are averaged)
// total value hast to be smaller than 2^16
#define ANALOG_MEASUREMENT_COUNT 10



uint16_t analog_measure_value(uint8_t analog_channel);