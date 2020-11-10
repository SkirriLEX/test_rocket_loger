#include <Arduino.h>
#include "MPU9250.h"

#include <BMP280_DEV.h> // Include the BMP280_DEV.h library

BMP280_DEV bmp280; // Instantiate (create) a BMP280_DEV object and set-up for I2C operation (address 0x77)
// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 IMU(Wire, 0x68);
int status;

void setup()
{
    // serial to display data
    Serial.begin(115200);
    while (!Serial)
    {
    }

    // start communication with IMU
    Wire.begin((uint8_t)PB11, (uint8_t)PB10);
    status = IMU.begin();
    if (status < 0)
    {
        Serial.println("IMU initialization unsuccessful");
        Serial.println("Check IMU wiring or try cycling power");
        Serial.print("Status: ");
        Serial.println(status);
        status = IMU.begin();
        delay(1000);
    }
    Serial.println(F("BMP280 Sensor event test"));
    while (!bmp280.begin())
    {
        Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
        delay(1000);
    }
    bmp280.setPresOversampling(OVERSAMPLING_X1); // Set the pressure oversampling to X4
    bmp280.setTempOversampling(OVERSAMPLING_X1); // Set the temperature oversampling to X1
    bmp280.setIIRFilter(IIR_FILTER_OFF);         // Set the IIR filter to setting 4
    bmp280.startNormalConversion();              // Start BMP280 forced conversion (if we're in SLEEP_MODE)

    // setting the accelerometer full scale range to +/-8G
    IMU.setAccelRange(MPU9250::ACCEL_RANGE_8G);
    // setting the gyroscope full scale range to +/-500 deg/s
    IMU.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
    // setting DLPF bandwidth to 20 Hz
    IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
    // setting SRD to 19 for a 50 Hz update rate
    IMU.setSrd(19);
}

void loop()
{
    // read the sensor
    IMU.readSensor();

    // display the data
    Serial.print(IMU.getAccelX_mss(), 6);
    Serial.print("\t");
    Serial.print(IMU.getAccelY_mss(), 6);
    Serial.print("\t");
    Serial.print(IMU.getAccelZ_mss(), 6);
    Serial.print("\t");
    Serial.print(IMU.getGyroX_rads(), 6);
    Serial.print("\t");
    Serial.print(IMU.getGyroY_rads(), 6);
    Serial.print("\t");
    Serial.print(IMU.getGyroZ_rads(), 6);
    Serial.print("\t");
    Serial.print(IMU.getMagX_uT(), 6);
    Serial.print("\t");
    Serial.print(IMU.getMagY_uT(), 6);
    Serial.print("\t");
    Serial.print(IMU.getMagZ_uT(), 6);
    Serial.print("\t");
    Serial.println(IMU.getTemperature_C(), 6);
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
    delay(1);
}