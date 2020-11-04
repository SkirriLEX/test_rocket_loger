#include <Arduino.h>
/////////////////////////////////////////////////////////////////////////////////
// BMP280_DEV - I2C Communications, Default Configuration, Normal Conversion
/////////////////////////////////////////////////////////////////////////////////

#include <BMP280_DEV.h> // Include the BMP280_DEV.h library

BMP280_DEV bmp280; // Instantiate (create) a BMP280_DEV object and set-up for I2C operation (address 0x77)

void setup()
{
  Serial.begin(115200);
  while (!Serial)
  {
    ; // wait for serial port to connect.
  }
  Serial.println(F("BMP280 Sensor event test"));
  Wire.begin((uint8_t)PB11, (uint8_t)PB10);
  while (!bmp280.begin())
  {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    delay(1000);
  }
  bmp280.setPresOversampling(OVERSAMPLING_X1); // Set the pressure oversampling to X4
  bmp280.setTempOversampling(OVERSAMPLING_X1); // Set the temperature oversampling to X1
  bmp280.setIIRFilter(IIR_FILTER_OFF);         // Set the IIR filter to setting 4
  bmp280.startNormalConversion();              // Start BMP280 forced conversion (if we're in SLEEP_MODE)
}

void loop()
{
  float temperature, pressure;                   // Create the temperature, pressure and altitude variables
  if (bmp280.getTempPres(temperature, pressure)) // Check if the measurement is complete
  {
    Serial.print(temperature); // Display the results
    Serial.print(F("*C   "));
    Serial.print(pressure);
    Serial.print(F("hPa   "));
    Serial.print(millis());
    Serial.print(F("\n"));
  }
}