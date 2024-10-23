# Automated chicken door opener

This is the third iteration of a seemingly simple task: Opening and closing a chicken door whenever the day begins and ends.
Uses a light sensitive resistor to check light level.

## avrdude Fuses

The following command was used to set the fuses on the Atmega328P

```bash
avrdude -c stk500v2 -p m328p -P /dev/ttyACM0 -U lfuse:w:0x62:m -U hfuse:w:0xd9:m -U efuse:w:0xff:m
```

This will set the default values and divide the clock by 8 to get a 1 MHz signal.

## Bitclock
If the chip is running below 4 MHz, one should set the bit clock in `platformio.ini`. 
It is the `-B 10.0` option.


