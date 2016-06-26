# Arduino-Test
Test sketches for Arduinos.

## Sercom
*For MKR1000 only* Test sketch with 4 serial ports:
- Serial  - Native USB interface
- Serial1 - Default serial port on D13, D14 (Sercom 5)
- Serial2 - Extra serial port on D0, D1 (Sercom 3)
- Serial3 - Extra serial port on D4, D5 (Sercom 4)

No more can be added with out reconfiguring the SPI or Wire interfaces.

## GY-87
  Tests basic functionality of the GY-87 sensor board. The board is small, cheap and contains an accelerometer, gyro, magnetometer and barometric pressure sensor. Requires the [I2Cdevlib](https://github.com/jrowberg/i2cdevlib) library.
  
