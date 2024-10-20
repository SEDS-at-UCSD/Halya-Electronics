#include "Arduino.h"
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_LSM6DS.h>
#include "SixDOF.h"
#include <Adafruit_Sensor.h>
#include "QuaternionFilter.h"
#define LSM_CS 35
#define LSM_SCK 36
#define LSM_MISO 37 // SDO
#define LSM_MOSI 38 // SDA
bool IGNITABLE = false;
double Net_Accel;
QuaternionFilter filter;

SixDOF::SixDOF()
{
}

Adafruit_LSM6DSOX Sensor = Adafruit_LSM6DSOX();

// Function definition
bool SixDOF::start_6DOF()
{
  Serial.println("Adafruit LSM6DSOX test!");
  if (!Sensor.begin_SPI(LSM_CS, LSM_SCK, LSM_MISO, LSM_MOSI))
  {
    Serial.println("Failed to find LSM6DSOX chip");
    return (false);
  }
  Serial.println("LSM6DSOX Found!");
  return true;
}

// Optional function definition (if needed in SixDOF.h)
String SixDOF::printSensorData()
{
  sensors_event_t accel1;
  sensors_event_t gyro1;
  sensors_event_t temp1;
  Sensor.getEvent(&accel1, &gyro1, &temp1);
  double Ax = accel1.acceleration.x;
  double Ay = accel1.acceleration.y;
  double Az = accel1.acceleration.z;
  Net_Accel = sqrt((pow(Ax, 2) + pow(Ay, 2) + pow(Az, 2)));
  return "(6DOF)" + String(Ax) + "," + String(Ay) + "," + String(Az) + "," + String(gyro1.gyro.x) + "," + String(gyro1.gyro.y) + "," + String(gyro1.gyro.z) + "," + "\n";
}

vector<double> SixDOF::getAcceleration()
{
  sensors_event_t accel1;
  return {accel1.acceleration.x, accel1.acceleration.y, accel1.acceleration.z};
}

vector<double> SixDOF::getGyro()
{
  sensors_event_t gyro1;
  return {static_cast<double>(gyro1.gyro.x), static_cast<double>(gyro1.gyro.y), static_cast<double>(gyro1.gyro.z)};
}

bool SixDOF::_init(int32_t sensor_id)
{
  return true; // Example return value, modify as needed
}

bool SixDOF::check_IGNITABLE()
{ // is there a way to use switch-cases here? Idk how to make cases for all values >10
  Serial.print("    Net Acceleration: " + String(Net_Accel) + ", ");
  if (Net_Accel > 10)
  {
    IGNITABLE = true;
  }
  else
  {
    switch (int(Net_Accel))
    {
    default:
      return IGNITABLE;
      break;
    }
  }
  return IGNITABLE;
}

// need to call manually in main.cpp file
void SixDOF::updateQuaternionFilter()
{
  vector<double> accelData = getAcceleration();
  vector<double> gyroData = getGyro();

  double gyroX = gyroData[0];
  double gyroY = gyroData[1];
  double gyroZ = gyroData[2];

  double accelX = accelData[0];
  double accelY = accelData[1];
  double accelZ = accelData[2];

  filter.update(accelX, accelY, accelZ, gyroX, gyroY, gyroZ, 0, 0, 0, quaternion);
}

vector<double> SixDOF::quaternionToEuler()
{
  float w = quaternion[0];
  float x = quaternion[1];
  float y = quaternion[2];
  float z = quaternion[3];
  float roll = atan2(2 * (x * w + y * z), 1 - 2 * (x * x + y * y));
  float pitch = asin(2 * (y * w - z * x));
  float yaw = atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
  return {static_cast<double>(roll), static_cast<double>(pitch), static_cast<double>(yaw)};
}
