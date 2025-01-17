#include <Arduino.h>
#include "PHT.h"
#include <Adafruit_MS8607.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>
#include <SimpleKalmanFilter.h>
#define SEALEVELPRESSURE_HPA (1013.25)

SimpleKalmanFilter altitudeFilter(1, 1, 0.1);

PHT::PHT()
{
  double altitude;
}

Adafruit_MS8607 Altimeter = Adafruit_MS8607();

TwoWire I2C_one(0);

bool PHT::startPHT()
{
  I2C_one.begin(42, 41);
  // 42 is SDA
  // 41 is SCL
  I2C_one.setClock(400000);
  if (!Altimeter.begin(&I2C_one))
  {
    Serial.println("Failed to find MS8607 chip");
    return false;
  }
  Serial.println("MS8607 Found, SETUP");
  return true;
}

String PHT::printReadings()
{
  sensors_event_t temp, pressure, humidity;
  Altimeter.getEvent(&pressure, &temp, &humidity);
  double pressure_alt = pressure.pressure;
  double altitude = 44330.0 * (1.0 - pow(pressure_alt / SEALEVELPRESSURE_HPA, 0.1903));
  return "(PHT) temp: " + String(temp.temperature) + " pressure: " + String(pressure.pressure) + " humidity: " + String(humidity.relative_humidity) + " altitude: " + String(altitude) + "\n";
}

double PHT::getAltitude()
{
  sensors_event_t temp, pressure, humidity;
  Altimeter.getEvent(&pressure, &temp, &humidity);
  double pressure_alt = pressure.pressure;
  double altitude = 44330.0 * (1.0 - pow(pressure_alt / SEALEVELPRESSURE_HPA, 0.1903));
  return altitude;
}

double PHT::getFilteredAltitude()
{
  altitude = getAltitude();
  filteredAltitude = alpha * altitude + (1 - alpha) * filteredAltitude;
  return filteredAltitude;
}