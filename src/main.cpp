#include <Arduino.h>
#include "MPU9250.h"
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Adafruit_LSM6DSOX.h>
#include "SixDOF.h"
#include "PHT.h"
#include <Adafruit_MS8607.h>
#include <math.h>
#include "GPS.h"
#include <map>
// #include "LaunchState.h"
#define SEALEVELPRESSURE_HPA (1013.25)
#define LSM_CS 5
#define LSM_SCK 18
#define LSM_MISO 19
#define LSM_MOSI 23
SixDOF _6DOF;
PHT Alt;
MPU9250 mpu;
TwoWire I2C_two(1);
uint16_t measurement_delay_us = 65535; // Delay between measurements for testing
GPS GPS1;
// LaunchState Halya;
int groundLevelAltitudeTest = 0;
std::map<string, bool> statesMapTest;
double AltArrayTest[10];
double previousMedianTest = 0;

bool CHECK = false;

double returnAverageTest(double arr[], int number)
{
  double sum = 0;
  for (int count = 0; count < number - 1; count++)
  {
    sum += arr[count];
  }
  return sum / number;
}

double returnMedianTest(double arr[], int number)
{
  double temp[number];
  memcpy(temp, arr, sizeof(temp));
  std::sort(temp, temp + number);

  if (number % 2 == 0)
  {
    return (temp[number / 2 - 2] + temp[number / 2 - 1] + temp[number / 2] + temp[number / 2 + 1]) / 4.0;
  }
  else
  {
    return (temp[number / 2 - 1] + temp[number / 2] + temp[number / 2 + 1] + temp[number / 2 + 2] + temp[number / 2 + 3]) / 5.0;
    ;
  }
}

double calculateRateOfChangeTest(double AltArray[], int READINGS_LENGTH)
{
  double currentMedian = returnMedianTest(AltArray, READINGS_LENGTH);

  double rate_of_change = currentMedian - previousMedianTest;

  previousMedianTest = currentMedian;

  return rate_of_change;
}

void setup()
{
  Serial.begin(115200);

  while (!Serial)
  {
    Serial.print("Serial Failed to start");
    delay(10);
  }

  GPS1.startGPS();
  delay(500);

  if (!(_6DOF.start_6DOF()))
  {
    Serial.println("6DOF Failed to start");
  }
  // Altimeter
  Alt.startPHT();
  // 9 DOF
  // I2C_two.begin(21, 22);
  // I2C_two.setClock(400000);
  // delay(1000);
  // if (!mpu.setup(0x68, MPU9250Setting(), I2C_two))
  // {
  //   while (1)
  //   {
  //     Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
  //     delay(1000);
  //   }
  // }
}

void loop()
{
  // // GPS

  // 6DOF
  Serial.print("6DOF READINGS\n");
  Serial.print(_6DOF.printSensorData());
  Serial.print("\n");
  // _6DOF.check_IGNITABLE();
  // vector<double> velocities = _6DOF.getVelocities();
  // Serial.print("\n");
  // delay(500);
  // vector<double> accels = _6DOF.getAcceleration();
  // Serial.print("\n");
  // vector<double> positions = _6DOF.getPositions();
  // Serial.print("\n");
  // for (int i = 0; i < velocities.size(); i++)
  // {
  //   Serial.print("Acceleration: " + String(accels[i]));
  //   Serial.print("\n");
  //   Serial.print("Velocity: " + String(velocities[i]));
  //   Serial.print("\n");
  //   Serial.print("Position: " + String(positions[i]));
  //   Serial.print("\n");
  // }
  // Serial.print(_6DOF.updateVerticalVelocity());
  // Serial.print("\n");
  // Serial.print(_6DOF.updateVerticalAltitude());
  // Serial.print("\n");
  _6DOF.updateVelocities();
  Serial.print("\n");
  _6DOF.updatePositions();
  Serial.print("\n");
  delay(500);

  // // Serial.print(String(_6DOF.check_IGNITABLE()) + "\n");
  // Altimeter
  // Serial.print("ALT READINGS\n");
  // Serial.print(Alt.printReadings());
  // Serial.print("\n");
  // Serial.print(Alt.getAltitude());
  // delay(500);
  // // 9DOF
  // // Serial.print("(9DOF)  "); // 9DOF tweaking
  // // if (mpu.update())
  // // {
  // //   static uint32_t prev_ms = millis();
  // //   Serial.print(String(mpu.getAcc(prev_ms)) + " ");
  // //   Serial.print(String(mpu.getGyro(prev_ms)) + "\n" + "\n");
  // // }
  // Serial.print("GPS READINGS\n");
  // GPS1.printInfo();
  // Serial.print(GPS1.getAltitude());
  // Serial.print("\n");
  // delay(500);
  // // Halya.HalyaStateMachine(_6DOF, Alt, mpu);
  // static int retryCount = 0;
  // const int MAX_RETRIES = 5;
  // _6DOF.check_IGNITABLE();

  // bool is_6DOF_working = _6DOF.checkReadings();
  // Serial.print(is_6DOF_working);
  // bool is_altimeter_working = Alt.getAltitude() != 0;
  // Serial.print(is_altimeter_working);
  // // bool is_mpu_working = mpu1.update() && mpu1.getAcc(millis()) != 0 & mpu1.getGyro(millis()) != 0;
  // bool is_gps_working = GPS1.readingCheck();
  // Serial.print(is_gps_working);

  // if (is_6DOF_working & is_altimeter_working & is_gps_working)
  // {
  //   groundLevelAltitudeTest = Alt.getAltitude();
  //   statesMapTest["_6DOF"] = true;
  //   statesMapTest["altimeter"] = true;
  //   statesMapTest["mpu"] = true;
  //   statesMapTest["gps"] = true;
  // }
  // else
  // {
  //   if (!is_6DOF_working)
  //     Serial.println("Warning: 6DOF sensor failure.");
  //   if (!is_altimeter_working)
  //     Serial.println("Warning: Altimeter failure.");

  //   delay(20);
  //   retryCount++;
  //   if (retryCount == MAX_RETRIES)
  //   {
  //     Serial.print("Continuing with limited functionality.");
  //     statesMapTest["_6DOF"] = is_6DOF_working;
  //     statesMapTest["altimeter"] = is_altimeter_working;
  //     statesMapTest["gps"] = is_gps_working;
  //   }
  // }

  // Serial.println("halya ignition!");

  // static int count = 0;

  // // Error flags for each of the sensors
  // bool PHT_error = (Alt.getAltitude() == 0);
  // // Serial.print(PHT_error);
  // bool GPS_error = GPS1.readingCheck();
  // // Serial.print(GPS_error);
  // bool IMU_error = !_6DOF.checkReadings();
  // // Serial.print(IMU_error);

  // double altReading = 0;

  // // Check if the PHT sensor is working
  // if (!PHT_error)
  // {
  //   altReading = Alt.getAltitude();
  //   // Serial.print(altReading);
  // }
  // // If the PHT sensor is not working, fall back to GPS
  // else if (!GPS_error)
  // {
  //   altReading = GPS1.getAltitude();
  //   // Serial.print(altReading);
  // }
  // // If both PHT and GPS are not working, fall back to IMU
  // else if (!IMU_error)
  // {
  //   altReading = _6DOF.updateVerticalAltitude();
  //   // Serial.print(altReading);
  // }

  // // Store the altitude reading in the circular buffer
  // AltArrayTest[count % 15] = altReading;
  // count++;

  // // Check for apogee based on the rate of change of altitude
  // if (count >= 15)
  // {
  //   double rate_of_change = calculateRateOfChangeTest(AltArrayTest, 15);
  //   Serial.print(rate_of_change);
  //   Serial.print("\n");
  //   if (fabs(rate_of_change) < 0.15 || rate_of_change <= 0)
  //   {
  //     // Serial.println("Halya has reached apogee!");
  //   }
  //   count = 0;
  // }

  // delay(500); // Delay for sensor updates
}

// Initialization: Initial state where sensors are set up and variables initialized.
// Pre-Launch: Reads sensor data, calculates altitude, and checks for ignition conditions.
// Ignition 1: Triggers ignition 1 and waits for a short delay.
// Ignition 2: Triggers ignition 2 (if certain conditions are met).
// Post-Launch: Reads sensor data and transmits telemetry (optional).
// Transitions between States:

// Initialization -> Pre-Launch: After successful initialization.
// Pre-Launch -> Pre-Launch: Stays in this state until ignition conditions are met.
// Pre-Launch -> Ignition 1: When IGNITABLE becomes true, rate_of_change is non-positive, and altitude is above 900 meters.
// Ignition 1 -> Ignition 2: After a short delay (1 second in your code).
// Ignition 2 (optional) -> Post-Launch: If a second ignition stage exists and its conditions are met.
// Pre-Launch/Ignition 1/Ignition 2 -> Error (optional): If any critical sensor readings fail or errors occur.
