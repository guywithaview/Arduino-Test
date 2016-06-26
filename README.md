# MKR1000-Test
Test sketches for the Arduino MKR1000.

## Sercom
Test sketch with 4 serial ports:
- Serial  - Native USB interface
- Serial1 - Default serial port on D13, D14 (Sercom 5)
- Serial2 - Extra serial port on D0, D1 (Sercom 3)
- Serial3 - Extra serial port on D4, D5 (Sercom 4)

No more can be added with out reconfiguring the SPI or Wire interfaces.
