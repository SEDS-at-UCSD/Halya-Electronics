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
#include <iostream>
#include <stdexcept>
#include <string>
// #include "LaunchState.h"
#define SEALEVELPRESSURE_HPA (1013.25)
#define LSM_CS 35
#define LSM_SCK 36
#define LSM_MISO 37 // SDO
#define LSM_MOSI 38 // SDA
SixDOF _6DOF;
PHT Alt;
MPU9250 mpu;
TwoWire I2C_two(1);
uint16_t measurement_delay_us = 65535; // Delay between measurements for testing
GPS GPS1;
// LaunchState Halya;

uint8_t integerPartToHex(double dataValue)
{
  int integerPart = static_cast<int>(dataValue);

  if (integerPart > 255)
  {
    integerPart = 255;
  }

  return 0x00 + static_cast<uint8_t>(integerPart);
}

uint8_t decimalPartToHex(double dataValue)
{
  int integerPart = static_cast<int>(dataValue);
  double decimalPart = dataValue - integerPart;

  int scaledDecimal = static_cast<int>(decimalPart * 100);

  // if (scaledDecimal > 255)
  // {
  //     scaledDecimal = 255;
  // }

  uint8_t hexValue = 0x00 + static_cast<uint8_t>(scaledDecimal);

  return hexValue;
}

double hextoDecimal(uint8_t hexadecimal_int, uint8_t hexadecimal_fraction)
{

  double integerResult = static_cast<double>(hexadecimal_int);

  double fractionResult = static_cast<double>(hexadecimal_fraction);

  return integerResult + fractionResult * (0.01);
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

  // if (!(_6DOF.start_6DOF()))
  // {
  //   Serial.println("6DOF Failed to start");
  // }
  // // Altimeter
  // Alt.startPHT();
  // // 9 DOF
  // I2C_two.begin(21, 22);
  // I2C_two.setClock(400000);
  // delay(1000);
  // if (!mpu.setup(0x68, MPU9250Setting(), I2C_two)) {
  //   while (1) {
  //     Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
  //     delay(1000);
  //   }
  // }
}

void loop()
{
  // GPS

  // 6DOF
  // Serial.print(_6DOF.printSensorData());
  // Serial.print(String(_6DOF.check_IGNITABLE()) + "\n");
  // // Altimeter
  // Serial.print(Alt.printReadings());
  // // 9DOF
  // Serial.print("(9DOF)  "); // 9DOF tweaking
  // if (mpu.update())
  // {
  //   static uint32_t prev_ms = millis();
  //   Serial.print(String(mpu.getAcc(prev_ms)) + " ");
  //   Serial.print(String(mpu.getGyro(prev_ms)) + "\n" + "\n");
  // }
  GPS1.printInfo();
  int32_t latitude = GPS1.latitude_fixed;
  int32_t longitude = GPS1.longitude_fixed;
  Serial.println(GPS1.convertToDegrees(latitude));
  Serial.println(GPS1.convertToDegrees(longitude));
  Serial.println(GPS1.representAsCoordinates(integerPartToHex(latitude), decimalPartToHex(latitude), GPS1.extraPrecision(latitude), integerPartToHex(longitude), decimalPartToHex(longitude), GPS1.extraPrecision(longitude)));
  delay(2250);
  // Serial.print("Hello World!");
  // Halya.HalyaStateMachine(_6DOF, Alt, mpu);
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
