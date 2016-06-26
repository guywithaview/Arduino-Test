/*
  GY-87 Test

  Tests basic functionality of the GY-87 sensor board.
  
  Connections:
  |== GY-87 ==|== Arduino ==|
  |  VCC_IN   |     VCC     |
  |   GND     |     GND     |
  |   SDA     |     SDA     |
  |   SCL     |     SCL     |
  |=========================|
 
  Requires the I2CDevLib library, which can be found here: https://github.com/jrowberg/i2cdevlib
  
  by Tom Kuehn
  26/06/2016
*/

#include "I2Cdev.h"
#include "MPU6050.h"
#include "HMC5883L.h"
#include "BMP085.h"
#include "Wire.h"

static const char LED = 6;
static const float ACCEL_SENS = 16384.0; // Accel Sensitivity with default +/- 2g scale
static const float GYRO_SENS  = 131.0;   // Gyro Sensitivity with default +/- 250 deg/s scale

// Magnetometer class default I2C address is 0x1E
// specific I2C addresses may be passed as a parameter here
// this device only supports one I2C address (0x1E)
HMC5883L mag;
int16_t mx, my, mz;

// Accel/Gyro class default I2C address is 0x68 (can be 0x69 if AD0 is high)
// specific I2C addresses may be passed as a parameter here
MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;

// Barometer class default I2C address is 0x77
// specific I2C addresses may be passed as a parameter here
// (though the BMP085 supports only one address)
BMP085 barometer;

float temperature;
float pressure;
int32_t lastMicros;

void setup()
{
  boolean state = HIGH;
  unsigned int count = 0;
  
  pinMode(LED, OUTPUT);
  
  Serial.begin(9600);
  while (!Serial && (count < 30) )
  {
    delay(200); // Wait for serial port to connect with timeout. Needed for native USB
    digitalWrite(LED, state);
    state = !state;
    count++;
  }

  digitalWrite(LED, HIGH);

  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();

  // ==================== MPU6050 ============================
  accelgyro.initialize();
  Serial.print("Testing Accel/Gyro... ");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  // Starts up with accel +/- 2 g and gyro +/- 250 deg/s scale
  accelgyro.setI2CBypassEnabled(true); // set bypass mode
  // Now we can talk to the HMC5883l

  // ==================== HMC5883L ============================
  mag.initialize();
  Serial.print("Testing Mag...  ");
  Serial.println(mag.testConnection() ? "HMC5883L connection successful" : "HMC5883L connection failed");

  // ==================== BMP085 ============================
  barometer.initialize();
  Serial.print("Testing Pressure...  ");
  Serial.println(barometer.testConnection() ? "BMP085 connection successful" : "BMP085 connection failed");

  Serial.println("Setup Complete");
}

void loop()
{
  static unsigned long ms = 0;
  static boolean state = HIGH;

  // Serial Output Format
  // === Accel === | === Gyro === | ======= Mag ======= | === Barometer === |
  //   X   Y   Z   |  X   Y   Z   |  X   Y   Z  Heading |  Temp   Pressure  |

  if (millis() - ms > 100)
  {
    // read raw accel/gyro measurements
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // display tab-separated accel/gyro x/y/z values
    Serial.print(ax/ACCEL_SENS); Serial.print("\t");
    Serial.print(ay/ACCEL_SENS); Serial.print("\t");
    Serial.print(az/ACCEL_SENS); Serial.print("\t");
    Serial.print(gx/GYRO_SENS); Serial.print("\t");
    Serial.print(gy/GYRO_SENS); Serial.print("\t");
    Serial.print(gz/GYRO_SENS); Serial.print("\t");

    // read raw heading measurements
    mag.getHeading(&mx, &my, &mz);

    // display tab-separated mag x/y/z values
    Serial.print(mx); Serial.print("\t");
    Serial.print(my); Serial.print("\t");
    Serial.print(mz); Serial.print("\t");
    
    // To calculate heading in degrees. 0 degree indicates North
    float heading = atan2(my, mx);
    if(heading < 0) heading += 2 * M_PI;
    Serial.print(heading * 180/M_PI); Serial.print("\t");

    // request temperature
    barometer.setControl(BMP085_MODE_TEMPERATURE);
    
    // wait appropriate time for conversion (4.5ms delay)
    lastMicros = micros();
    while (micros() - lastMicros < barometer.getMeasureDelayMicroseconds());

    // read calibrated temperature value in degrees Celsius
    temperature = barometer.getTemperatureC();

    // request pressure (3x oversampling mode, high detail, 23.5ms delay)
    barometer.setControl(BMP085_MODE_PRESSURE_3);
    while (micros() - lastMicros < barometer.getMeasureDelayMicroseconds());

    // read calibrated pressure value in Pascals (Pa)
    pressure = barometer.getPressure();

    // display measured values if appropriate
    Serial.print(temperature); Serial.print("\t");
    Serial.print(pressure/100); Serial.println("\t");

    ms = millis();
    digitalWrite(LED, state);
    state = !state;
  }
}
