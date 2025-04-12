#include <Arduino.h>
// #include "MPU9250.h"
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Adafruit_LSM6DSOX.h>
#include "SixDOF.h"
#include "PHT.h"
#include <Adafruit_MS8607.h>
#include <math.h>
#include "GPS.h"
#include "NineDOF.h"
#include <map>
#include "esp_system.h"
#include "esp_adc_cal.h"
#include "driver/temp_sensor.h"
// #include "LaunchState.h"
#define SEALEVELPRESSURE_HPA (1013.25)
#define LSM_CS 5
#define LSM_SCK 18
#define LSM_MISO 19
#define LSM_MOSI 23
SixDOF _6DOF;
// PHT Alt;
// MPU9250 mpu;
// TwoWire I2C_n(1);
TwoWire I2C_one(0);
PHT Alt(I2C_one);
uint16_t measurement_delay_us = 65535; // Delay between measurements for testing
GPS GPS1;
// LaunchState Halya;
NineDOF _9DOF;
int groundLevelAltitudeTest = 0;
// std::map<string, bool> statesMapTest;
double AltArrayTest[10];
double previousMedianTest = 0;

bool CHECK = false;

float getCoreTemperature()
{
  float temp_value = 0;

  // Initialize temperature sensor
  temp_sensor_config_t temp_sensor = TSENS_CONFIG_DEFAULT();
  temp_sensor_get_config(&temp_sensor);
  temp_sensor.dac_offset = TSENS_DAC_L2; // Adjust DAC offset if needed

  temp_sensor_set_config(temp_sensor);
  temp_sensor_start();

  // Read temperature
  temp_sensor_read_celsius(&temp_value);
  temp_sensor_stop();

  return temp_value;
}

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
  Serial.begin(921600);

  // temp_sensor_start();

  while (!Serial)
  {
    Serial.print("Serial Failed to start");
    delay(10);
  }

  // GPS1.startGPS();
  // delay(500);

  // if (!(_6DOF.start_6DOF()))
  // {
  //   Serial.println("6DOF Failed to start");
  // }
  // Altimeter
  // Alt.startPHT();
  I2C_one.begin(42, 41);
  if (!Alt.connectSensor())
  {
    Serial.println("Error connecting to sensor...");
  }
  else
  {
    Serial.println("Connected to sensor");
    Alt.setSensorConfig();
  }
  // 9 DOF
  // I2C_two.begin(36, 37);
  // if (!_9DOF.begin())
  // {
  //   Serial.println("Sensor initialization failed!");
  //   while (1)
  //     delay(10);
  // }
  // _9DOF.calibrateSensors();
  // _9DOF.calibrateMag();

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

static int counter = 0;

void loop()
{

  float coreTemp = getCoreTemperature();
  Serial.print("ESP32-S3 Core Temperature: ");
  Serial.print(coreTemp);
  Serial.println(" °C");

  // // GPS
  // GPS1.printInfo();
  // nmea_float_t latitude = GPS1.latitude;
  // double degrees = GPS1.convertToDegrees(latitude);
  // Serial.println(degrees);

  // 6DOF
  // Serial.print("6DOF READINGS\n");
  // Serial.print(_6DOF.printSensorData());
  // Serial.print("\n");
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
  // _6DOF.updateVelocities();
  // Serial.print("\n");
  // _6DOF.updatePositions();
  // Serial.print("\n");
  // delay(500);

  // Serial.print(String(_6DOF.check_IGNITABLE()) + "\n");
  // Altimeter
  Alt.updateData();
  Alt.printData();
  Serial.println(String(Alt.getAltitude()) + " meters \n");
  // delay(1000);
  // delay(500);

  delay(500); // Delay for sensor updates
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