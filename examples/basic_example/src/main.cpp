#include <Arduino.h>
#include <Wire.h>

#include <MUFFINS__MS56XX.h>

const int SENSOR_POWER_ENABLE_PIN = 17;
const int WIRE0_SCL = 1;
const int WIRE0_SDA = 0;

MS56XX ms5611;

MS56XX::Config config = {
  .wire = &Wire,
  .i2c_address = MS56XX::I2C_0x77,
  .type = MS56XX::MS5611,
  .oversampling = MS56XX::OSR_ULTRA_HIGH,
  .reference_pressure = 101325
};

MS56XX::Data data;

void setup()
{
  Serial.begin(115200);
  while (!Serial)
  {
    delay(1000);
  }

  pinMode(SENSOR_POWER_ENABLE_PIN, OUTPUT_12MA);
  digitalWrite(SENSOR_POWER_ENABLE_PIN, HIGH);

  if (Wire.setSCL(WIRE0_SCL) && Wire.setSDA(WIRE0_SDA))
  {
    Wire.begin();
  }

  if (!ms5611.begin(config))
  {
    while(1);
  }
}

void loop()
{
  if (ms5611.read(data))
  {
    Serial.print("Temperature: ");
    Serial.print(data.temperature);
    Serial.print(" C, Pressure: ");
    Serial.print(data.pressure);
    Serial.print(" Pa, Altitude: ");
    Serial.print(data.altitude);
    Serial.println(" m");
  }
  else
  {
    Serial.println("Error reading sensor data");
  }

  delay(100);
}