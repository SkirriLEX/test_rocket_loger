#include <Arduino.h>

#include "MPU9250.h"
#include "BMP280_DEV.h"

BMP280_DEV bmp280;       //(address 0x77)
MPU9250 MPU(Wire, 0x68); //(address 0x68)
int status;

void setup()
{
    Serial.begin(115200);
    while (!Serial)
    {
        ;
    }

    Wire.begin((uint8_t)PB11, (uint8_t)PB10);
    status = MPU.begin();
    Serial.println("MPU Sensor event test");
    if (status < 0)
    {
        Serial.println("MPU initialization unsuccessful");
        Serial.println("Check MPU wiring or try cycling power");
        Serial.print("Status: ");
        Serial.println(status);
        status = MPU.begin();
        while (true)
            ;
    }
    Serial.println(F("BMP280 Sensor event test"));
    if (!bmp280.begin(0x76))
    {
        Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
        while (true)
            ;
    }
    Wire.setClock(400000);
    bmp280.setPresOversampling(OVERSAMPLING_X1); // Set the pressure oversampling to X4
    bmp280.setTempOversampling(OVERSAMPLING_X1); // Set the temperature oversampling to X1
    bmp280.setIIRFilter(IIR_FILTER_OFF);         // Set the IIR filter to setting 4
    bmp280.startNormalConversion();              // Start BMP280 forced conversion (if we're in SLEEP_MODE)

    MPU.setAccelRange(MPU9250::ACCEL_RANGE_16G);
    MPU.setGyroRange(MPU9250::GYRO_RANGE_2000DPS);
    MPU.setSrd(0);
    MPU.enableDataReadyInterrupt();

    Serial.println("init done");
}

void loop()
{
    if (MPU.readSensor() > 0)
    {
        Serial.print(MPU._axcounts);
        Serial.print("\t");
        Serial.print(MPU._aycounts);
        Serial.print("\t");
        Serial.print(MPU._azcounts);
        Serial.print("\t");
        Serial.print(MPU._gxcounts);
        Serial.print("\t");
        Serial.print(MPU._gycounts);
        Serial.print("\t");
        Serial.print(MPU._gzcounts);
        Serial.print("\t");
        Serial.print(MPU._hxcounts);
        Serial.print("\t");
        Serial.print(MPU._hycounts);
        Serial.print("\t");
        Serial.print(MPU._hzcounts);
        Serial.print("\t");
        Serial.print(MPU._tcounts);
        Serial.print("\t");
        Serial.print(micros());
        Serial.print("\n");
    }
    float temperature, pressure;                   // Create the temperature, pressure and altitude variables
    if (bmp280.getTempPres(temperature, pressure)) // Check if the measurement is complete
    {
        Serial.print(temperature);
        Serial.print("\t");
        Serial.print(pressure);
        Serial.print("\t");
        Serial.print(micros());
        Serial.print("\n");
    }
}