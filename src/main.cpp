#include <Arduino.h>



/*
Advanced_I2C.ino
Brian R Taylor
brian.taylor@bolderflight.com

Copyright (c) 2017 Bolder Flight Systems

Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
and associated documentation files (the "Software"), to deal in the Software without restriction, 
including without limitation the rights to use, copy, modify, merge, publish, distribute, 
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is 
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or 
substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, 
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

// #include "MPU9250.h"

// // an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
// MPU9250 IMU(Wire,0x68);
// int status;

// void setup() {
//   // serial to display data
//   Serial.begin(115200);
//   while(!Serial) {}

//   // start communication with IMU 
//   status = IMU.begin();
//   if (status < 0) {
//     Serial.println("IMU initialization unsuccessful");
//     Serial.println("Check IMU wiring or try cycling power");
//     Serial.print("Status: ");
//     Serial.println(status);
//     status = IMU.begin();
//     delay (1000);
//   }
//   // setting the accelerometer full scale range to +/-8G 
//   IMU.setAccelRange(MPU9250::ACCEL_RANGE_8G);
//   // setting the gyroscope full scale range to +/-500 deg/s
//   IMU.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
//   // setting DLPF bandwidth to 20 Hz
//   IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
//   // setting SRD to 19 for a 50 Hz update rate
//   IMU.setSrd(19);
// }

// void loop() {
//   // read the sensor
//   IMU.readSensor();

//   // display the data
//   Serial.print(IMU.getAccelX_mss(),6);
//   Serial.print("\t");
//   Serial.print(IMU.getAccelY_mss(),6);
//   Serial.print("\t");
//   Serial.print(IMU.getAccelZ_mss(),6);
//   Serial.print("\t");
//   Serial.print(IMU.getGyroX_rads(),6);
//   Serial.print("\t");
//   Serial.print(IMU.getGyroY_rads(),6);
//   Serial.print("\t");
//   Serial.print(IMU.getGyroZ_rads(),6);
//   Serial.print("\t");
//   Serial.print(IMU.getMagX_uT(),6);
//   Serial.print("\t");
//   Serial.print(IMU.getMagY_uT(),6);
//   Serial.print("\t");
//   Serial.print(IMU.getMagZ_uT(),6);
//   Serial.print("\t");
//   Serial.println(IMU.getTemperature_C(),6);
//   delay(20);
// }
/////////////////////////////////////////////////////////////////////////////////
// BMP280_DEV - I2C Communications, Default Configuration, Normal Conversion
/////////////////////////////////////////////////////////////////////////////////

// #include <BMP280_DEV.h> // Include the BMP280_DEV.h library

// BMP280_DEV bmp280; // Instantiate (create) a BMP280_DEV object and set-up for I2C operation (address 0x77)

// void setup()
// {
//   Serial.begin(115200);
//   while (!Serial)
//   {
//     ; // wait for serial port to connect.
//   }
//   Serial.println(F("BMP280 Sensor event test"));
//   Wire.begin((uint8_t)PB11, (uint8_t)PB10);
//   while (!bmp280.begin())
//   {
//     Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
//     delay(1000);
//   }
//   bmp280.setPresOversampling(OVERSAMPLING_X1); // Set the pressure oversampling to X4
//   bmp280.setTempOversampling(OVERSAMPLING_X1); // Set the temperature oversampling to X1
//   bmp280.setIIRFilter(IIR_FILTER_OFF);         // Set the IIR filter to setting 4
//   bmp280.startNormalConversion();              // Start BMP280 forced conversion (if we're in SLEEP_MODE)
// }

// void loop()
// {
//   float temperature, pressure;                   // Create the temperature, pressure and altitude variables
//   if (bmp280.getTempPres(temperature, pressure)) // Check if the measurement is complete
//   {
//     Serial.print(temperature); // Display the results
//     Serial.print(F("*C   "));
//     Serial.print(pressure);
//     Serial.print(F("hPa   "));
//     Serial.print(millis());
//     Serial.print(F("\n"));
//   }
// }