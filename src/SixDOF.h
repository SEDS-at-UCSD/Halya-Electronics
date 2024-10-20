#ifndef _SixDOF_H
#include "Arduino.h"
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_LSM6DS.h>
#include <Adafruit_Sensor.h>
#define LSM_CS 35
#define LSM_SCK 36
#define LSM_MISO 37 // SDO
#define LSM_MOSI 38 // SDA
#include <vector>
using namespace std;

class SixDOF : public Adafruit_LSM6DSOX
{
public:
  float quaternion[4];
  SixDOF();
  bool start_6DOF();
  String printSensorData();
  double Net_Accel;
  bool IGNITABLE;
  bool check_IGNITABLE();
  vector<double> getAcceleration();
  vector<double> getGyro();
  void updateQuaternionFilter();
  vector<double> quaternionToEuler();

private:
  bool _init(int32_t sensor_id);
};

#endif