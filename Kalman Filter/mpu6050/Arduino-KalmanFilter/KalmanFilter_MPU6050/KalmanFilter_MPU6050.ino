// Include libraries
#include <Wire.h>
#include <Kalman.h> // Kalman filter library (install from Library Manager or GitHub)

// MPU6050 I2C address
#define MPU6050_ADDR 0x68

// MPU6050 register addresses
#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H 0x43
#define PWR_MGMT_1 0x6B

// Kalman filters for roll and pitch
Kalman kalmanRoll;
Kalman kalmanPitch;

// Time variables
unsigned long prevTime;
double dt;

// Raw sensor data variables
int16_t accX, accY, accZ;
int16_t gyroX, gyroY, gyroZ;

// Angles
double roll, pitch;
double filteredRoll, filteredPitch;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  // Initialize MPU6050
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(PWR_MGMT_1);
  Wire.write(0);
  Wire.endTransmission(true);

  // Wait for sensor to stabilize
  delay(100);

  // Initialize time
  prevTime = millis();
}

void loop() {
  // Calculate dt
  unsigned long currentTime = millis();
  dt = (currentTime - prevTime) / 1000.0;
  prevTime = currentTime;

  // Read raw accelerometer data
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 6, true);
  accX = (Wire.read() << 8) | Wire.read();
  accY = (Wire.read() << 8) | Wire.read();
  accZ = (Wire.read() << 8) | Wire.read();

  // Read raw gyroscope data
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(GYRO_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 6, true);
  gyroX = (Wire.read() << 8) | Wire.read();
  gyroY = (Wire.read() << 8) | Wire.read();
  gyroZ = (Wire.read() << 8) | Wire.read();

  // Convert accelerometer data to angles
  roll = atan2(accY, accZ) * 180 / PI;
  pitch = atan2(-accX, sqrt(accY * accY + accZ * accZ)) * 180 / PI;

  // Convert gyroscope data to deg/s
  double gyroXrate = gyroX / 131.0;
  double gyroYrate = gyroY / 131.0;

  // Apply Kalman filter
  filteredRoll = kalmanRoll.getAngle(roll, gyroXrate, dt);
  filteredPitch = kalmanPitch.getAngle(pitch, gyroYrate, dt);

  // Print results
  Serial.print("Filtered Roll: ");
  Serial.print(filteredRoll);
  Serial.print("\tFiltered Pitch: ");
  Serial.println(filteredPitch);

  delay(10);
}
