
/*
  SERCOM Test
  
  Test the ability to add extra hardware serial ports to the MKR1000
  This sketch has the following serial interfaces:
    Serial  - Native USB interface
    Serial1 - Default serial port on D13, D14 (Sercom 5)
    Serial2 - Extra serial port on D0, D1 (Sercom 3)
    Serial3 - Extra serial port on D4, D5 (Sercom 4)
    
  Good explanation of sercom funcationality here: 
  https://learn.adafruit.com/using-atsamd21-sercom-to-add-more-spi-i2c-serial-ports/muxing-it-up

  for Arduino MKR1000
  by Tom Kuehn
  26/06/2016

*/

#include <Arduino.h>                              // required before wiring_private.h
#include <wiring_private.h>

static const char MKR1000_LED       = 6;

// Serial2 pin and pad definitions (in Arduino files Variant.h & Variant.cpp)
#define PIN_SERIAL2_RX       (1ul)                // Pin description number for PIO_SERCOM on D1
#define PIN_SERIAL2_TX       (0ul)                // Pin description number for PIO_SERCOM on D0
#define PAD_SERIAL2_TX       (UART_TX_PAD_0)      // SERCOM pad 0 TX
#define PAD_SERIAL2_RX       (SERCOM_RX_PAD_1)    // SERCOM pad 1 RX

// Serial3 pin and pad definitions (in Arduino files Variant.h & Variant.cpp)
#define PIN_SERIAL3_RX       (5ul)                // Pin description number for PIO_SERCOM on D5
#define PIN_SERIAL3_TX       (4ul)                // Pin description number for PIO_SERCOM on D4
#define PAD_SERIAL3_TX       (UART_TX_PAD_2)      // SERCOM pad 2 TX
#define PAD_SERIAL3_RX       (SERCOM_RX_PAD_3)    // SERCOM pad 3 RX

// Instantiate the extra Serial classes
Uart Serial2(&sercom3, PIN_SERIAL2_RX, PIN_SERIAL2_TX, PAD_SERIAL2_RX, PAD_SERIAL2_TX);
Uart Serial3(&sercom4, PIN_SERIAL3_RX, PIN_SERIAL3_TX, PAD_SERIAL3_RX, PAD_SERIAL3_TX);

void SERCOM3_Handler()    // Interrupt handler for SERCOM3
{
  Serial2.IrqHandler();
}

void SERCOM4_Handler()    // Interrupt handler for SERCOM4
{
  Serial3.IrqHandler();
}

void setup()
{
  static boolean state = HIGH;
  static unsigned char count = 0;
  
  pinMode(MKR1000_LED, OUTPUT);
  Serial1.begin(9600);
   
  // Start Serial for debugging on the Serial Monitor
  Serial.begin(9600);
  while (!Serial && (count < 30) )
  {
    delay(200); // wait for serial port to connect. Needed for native USB
    digitalWrite(MKR1000_LED, state);
    state = !state;
    count++;
  }

  pinPeripheral(0, PIO_SERCOM);   // Assign pins 0 & 1 SERCOM functionality
  pinPeripheral(1, PIO_SERCOM);
  Serial2.begin(57600);           // Begin Serial2 

  pinPeripheral(4, PIO_SERCOM_ALT);   // Assign pins 4 & 5 SERCOM functionality
  pinPeripheral(5, PIO_SERCOM_ALT);
  Serial3.begin(57600);               // Begin Serial3
  
  digitalWrite(MKR1000_LED, HIGH);

  Serial.println("Setup Complete");
  Serial1.println("Setup Complete");
  Serial2.println("Setup Complete");
  Serial3.println("Setup Complete");
}

void loop()
{
  static unsigned long ms = 0;
  static boolean state = HIGH;
  static unsigned char c = 0;

  if (Serial.available())
  {
    char c = Serial.read();
    Serial.print(c);
    Serial1.print(c);
    Serial2.print(c);
    Serial3.print(c);
  }
  if (Serial1.available())
  {
    char c = Serial1.read();
    Serial.print(c);
    Serial1.print(c);
    Serial2.print(c);
    Serial3.print(c);
  }
  if (Serial2.available())
  {
    char c = Serial2.read();
    Serial.print(c);
    Serial1.print(c);
    Serial2.print(c);
    Serial3.print(c);
  }
  if (Serial3.available())
  {
    char c = Serial3.read();
    Serial.print(c);
    Serial1.print(c);
    Serial2.print(c);
    Serial3.print(c);
  }


  if (millis() - ms > 100)
  {
    ms = millis();
    digitalWrite(MKR1000_LED, state);
    state = !state;
  }

}
