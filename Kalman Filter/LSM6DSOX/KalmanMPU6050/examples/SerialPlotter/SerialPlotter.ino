//For wiring of 6DOF, check https://learn.adafruit.com/lsm6dsox-and-ism330dhc-6-dof-imu/arduino
//Code can be used for any IMU. for communication protocol changes, edit the CPP file.

//The header file has MPU6050 in the name because I was lazy to change it. its set up for the LSM6DSOX
#include "KalmanMPU6050.h"

long then = 0;
// Define the circular buffer and its size for exponentially moving average filter
#define BUFFER_SIZE 10
float rollBuffer[BUFFER_SIZE] = {0}; // Circular buffer for roll values
float pitchBuffer[BUFFER_SIZE] = {0}; // Circular buffer for pitch values
int bufferIndex = 0;                 // Index to keep track of circular buffer position

float rollSum = 0;                   // Sum of the roll buffer values
float pitchSum = 0;                  // Sum of the pitch buffer values

void setup()
{
  Serial.begin(115200);

  IMU::init();
  IMU::read();

  pinMode(LED_BUILTIN, OUTPUT);
}

void loop()
{
  /* Reads the data from the MPU and processes it with the Kalman Filter */
  IMU::read();

  // Get current roll and pitch values
  float currentRoll = IMU::getRoll();
  float currentPitch = IMU::getPitch();

  // Update the circular buffer for roll
  rollSum -= rollBuffer[bufferIndex];        // Subtract the oldest value from the sum
  rollBuffer[bufferIndex] = currentRoll;     // Replace it with the new value
  rollSum += rollBuffer[bufferIndex];        // Add the new value to the sum

  // Update the circular buffer for pitch
  pitchSum -= pitchBuffer[bufferIndex];      // Subtract the oldest value from the sum
  pitchBuffer[bufferIndex] = currentPitch;   // Replace it with the new value
  pitchSum += pitchBuffer[bufferIndex];      // Add the new value to the sum

  // Move to the next index in the circular buffer
  bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;

  // Calculate the moving averages
  float rollAvg = rollSum / BUFFER_SIZE;
  float pitchAvg = pitchSum / BUFFER_SIZE;

  // Subtract the moving average from the current values
  float rollFiltered = currentRoll - rollAvg;
  float pitchFiltered = currentPitch - pitchAvg;


  Serial.print(rollFiltered);
  Serial.print(",");
  Serial.print(pitchFiltered);
  Serial.print(",");
  Serial.print(IMU::getRoll());
  Serial.print(",");
  Serial.println(IMU::getPitch());

  if (millis() - then >= 1000)
  {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    then = millis();
  }
}