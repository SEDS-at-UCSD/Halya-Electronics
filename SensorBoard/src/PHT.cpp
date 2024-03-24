#include <Arduino.h>
#include "PHT.h"
#include <Adafruit_MS8607.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>
#define SEALEVELPRESSURE_HPA (1013.25)

PHT::PHT() {   
}

Adafruit_MS8607 Altimeter = Adafruit_MS8607();

TwoWire I2C_one(0);

bool PHT::startPHT(){
    I2C_one.begin(13, 14);
    I2C_one.setClock(400000);
    if (!Altimeter.begin(&I2C_one)) {
        Serial.println("Failed to find MS8607 chip");
            return false;

  }
  Serial.println("MS8607 Found, SETUP");
    return true;
}

String PHT::printReadings(){
  sensors_event_t temp, pressure, humidity;
  Altimeter.getEvent(&pressure, &temp, &humidity);
  double pressure_alt = pressure.pressure;
  double altitude = 44330.0 * (1.0 - pow(pressure_alt / SEALEVELPRESSURE_HPA, 0.1903));
  return "temp: " + String(temp.temperature) + " pressure: " + String(pressure.pressure) + " humidity: " + String(humidity.relative_humidity) + " altitude: " + String(altitude) + "\n";

}


// bool PHT::IGNITABLE() {
// switch (true) {
//     case():
// }
// }