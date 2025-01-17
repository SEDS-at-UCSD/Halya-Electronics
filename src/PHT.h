#ifndef _PHT_H
#include <Arduino.h>
#include <Adafruit_MS8607.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
class PHT : public Adafruit_MS8607
{
public:
    PHT();
    bool startPHT();
    String printReadings();
    double getAltitude();
    double getFilteredAltitude();

private:
    double altitude;
    double filteredAltitude;
    const double alpha = 0.1;
};

#endif