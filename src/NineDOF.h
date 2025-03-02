#ifndef _NineDOF_H

#include <Wire.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <vector>
#include <MadgwickAHRS.h>
using namespace std;

class NineDOF
{
public:
    NineDOF();
    bool begin();
    void calibrateSensors();
    void calibrateMag();
    void readSensorData();
    // bool check_IGNITABLE();
    vector<double> getAcceleration();
    vector<double> getGyro();
    vector<double> getMagno();
    double getNetAccel();
    // vector<double> getOrientation();
    // void updateFilter();
    // vector<double> transformToWorldFrame();
    // double updateVerticalMotion(double dt);

private:
    Adafruit_ICM20948 icm;
    TwoWire I2C_9dof;

    Madgwick filter;
};

#endif
