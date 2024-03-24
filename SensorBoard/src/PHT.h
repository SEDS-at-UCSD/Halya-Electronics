#ifndef _PHT_H
#include <Arduino.h>
#include <Adafruit_MS8607.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
class PHT : public Adafruit_MS8607 {
    public:
        PHT();
        bool startPHT();
        String printReadings();
        // bool IGNITABLE();
};

#endif