#ifndef _NineDOF_H
#include "Arduino.h"
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#define LSM_CS
#define LSM_SCK
#define LSM_MISO
#define LSM_MOSI
#include <vector>
using namespace std;

class NineDOF : public Adafruit_ICM20948
{
public:
    NineDOF();
    bool start_9DOF();
    String printSensorData();
    bool IGNITABLE;
    bool check_IGNITABLE();
    vector<double> getAcceleration();
    vector<double> getGyro();
    vector<double> getMag();
    bool checkReadings();
    double updateVerticalVelocity();
    double updateVerticalAltitude();
    void updateVelocities();
    void updatePositions();
    vector<double> getVelocities();
    vector<double> getPositions();

private:
    bool _init(int32_t sensor_id);
};
#endif