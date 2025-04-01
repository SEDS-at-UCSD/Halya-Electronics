#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// Create sensor instance
Adafruit_ICM20948 icm;

// Initialize sensors
bool init_sensors(void) {
  if (!icm.begin_I2C()) {
    return false;
  }

  // Assign pointers to sensor interfaces
  accelerometer = icm.getAccelerometerSensor();
  gyroscope = icm.getGyroSensor();
  magnetometer = icm.getMagnetometerSensor();

  return true;
}

void setup_sensors(void) {
  // Optional: Customize sensor config if desired
  // icm.setAccelRange(ICM20948_ACCEL_RANGE_4_G);
  // icm.setGyroRange(ICM20948_GYRO_RANGE_500_DPS);
  // icm.setMagDataRate(AK09916_MAG_DATARATE_50_HZ);

  // Default is fine for MotionCal
}
