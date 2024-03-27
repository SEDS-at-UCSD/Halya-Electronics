#include "Arduino.h"
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_LSM6DS.h>
#include "SixDOF.h"
#include <Adafruit_Sensor.h>
#define LSM_CS 5
#define LSM_SCK 18
#define LSM_MISO 19
#define LSM_MOSI 23
bool IGNITABLE = false;
double Net_Accel;

SixDOF::SixDOF() {
}

Adafruit_LSM6DSOX Sensor = Adafruit_LSM6DSOX();

// Function definition
bool SixDOF::start_6DOF() {
  Serial.println("Adafruit LSM6DSOX test!");
  if (!Sensor.begin_SPI(LSM_CS, LSM_SCK, LSM_MISO, LSM_MOSI)) {
    Serial.println("Failed to find LSM6DSOX chip");
    return(false);
  }
  Serial.println("LSM6DSOX Found!");
  return true;
}

// Optional function definition (if needed in SixDOF.h)
String SixDOF::printSensorData() {
  sensors_event_t accel1;
  sensors_event_t gyro1;
  sensors_event_t temp1;
  Sensor.getEvent(&accel1, &gyro1, &temp1);
  double Ax = accel1.acceleration.x;
  double Ay = accel1.acceleration.y;
  double Az = accel1.acceleration.z;
  Net_Accel = sqrt((pow(Ax, 2) + pow(Ay, 2) + pow(Az, 2)));
  return "(6DOF)" + String(Ax) + "," + String(Ay) + "," + String(Az) + "," + String(gyro1.gyro.x) + ","+ String(gyro1.gyro.y) + ","+ String(gyro1.gyro.z) + ","+ "\n";
}

bool SixDOF::_init(int32_t sensor_id) {
  return true; // Example return value, modify as needed
}

bool SixDOF::check_IGNITABLE(){     //is there a way to use switch-cases here? Idk how to make cases for all values >10
  Serial.print("    Net Acceleration: " +String(Net_Accel)+ ", ");
    if(Net_Accel > 10 ) {
      IGNITABLE = true;
    }
    else {
      switch (int(Net_Accel)) {
        default:
        return IGNITABLE;
        break;  
      }
    }
    return IGNITABLE;
}
