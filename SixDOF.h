#ifndef _SixDOF_H
#include "Arduino.h"
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_LSM6DS.h>
#include <Adafruit_Sensor.h>
#define LSM_CS 5
#define LSM_SCK 18
#define LSM_MISO 19
#define LSM_MOSI 23

class SixDOF : public Adafruit_LSM6DSOX {
  public:
    SixDOF();
    bool start_6DOF();
    String printSensorData();
    double Net_Accel;
    bool IGNITABLE;
    bool check_IGNITABLE();

private:
  bool _init(int32_t sensor_id);
};

#endif