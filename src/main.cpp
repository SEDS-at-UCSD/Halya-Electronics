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
#include <map>
#include <ArduinoJson.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/twai.h"
#include <task.h>
#include <queue.h>
#include "LaunchState.h"
#include <ESP32-TWAI-CAN.hpp>
// Task handles
GPS GPS_NAV;
TaskHandle_t transmitTaskHandle;
TaskHandle_t receiveTaskHandle;
String canMessage;
int counter = 0;
volatile bool pinStatusUpdated[64] = {};
volatile int pinStatus[64][5] = {};
StaticJsonDocument<512> sensorDataGlobal;
SemaphoreHandle_t mutex_d; // dataserialize
// CAN TWAI message to send
twai_message_t txMessage;
CanFrame rxFrame;
int canTXRXcount[2] = {0, 0};
// #include "LaunchState.h"
#define SEALEVELPRESSURE_HPA (1013.25)
#define LSM_CS 38
#define LSM_SCK 37
#define LSM_MISO 35
#define LSM_MOSI 36

#define PHT_SDA 42
#define PHT_SCL 41

#define ICM_SDA 39
#define ICM_SCL 40

// Function prototypes
void transmitTask(void *pvParameters);
void receiveTask(void *pvParameters);
void commandTask(void *pvParameters);
String messageToCAN(String code);
// Define CAN pins
#define CAN_TX 16
#define CAN_RX 17
LaunchState current_state1 = LaunchState::PreIgnition;
SixDOF _6DOF;
// MPU9250 mpu;
TwoWire I2C1(0); // Default I2C bus
PHT Alt(I2C1);
TwoWire I2C2(1);                       // Secondary I2C bus
uint16_t measurement_delay_us = 65535; // Delay between measurements for testing
GPS GPS1;
// LaunchState Halya;
int groundLevelAltitudeTest = 0;
std::map<string, bool> statesMapTest;
double AltArrayTest[10];
double previousMedianTest = 0;
bool CHECK = false;
const int baudrate = 921600;
const int rows = 4;
const int cols = 50;
int sendCAN = 1;
// printing helper variables
int waitforADS = 0;
int printADS[4] = {1, 1, 1, 1};
int printCAN = 1;
String loopprint;
int cycledelay = 2;
int preignitionCount = 0;
int expectedAltitude = 160;
int startingAltitude = 0;
int offset = 0;
// Define a queue to hold the JSON data
QueueHandle_t jsonQueue;
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

uint8_t tens_ones_digits(double dataValue)
{
  int integerPart = static_cast<int>(dataValue);
  int first_two = integerPart % 100;
  //  if (integerPart > 255)
  //    {
  //        integerPart = 255;
  //    }
  return 0x00 + static_cast<uint8_t>(first_two);
}
uint8_t thous_hunds_digits(double dataValue)
{
  int integerPart = static_cast<int>(dataValue);
  int last_two = integerPart / 100;
  //  if (integerPart > 255)
  //    {
  //        integerPart = 255;
  //    }
  return 0x00 + static_cast<uint8_t>(last_two);
}
double hextoDecimal(uint8_t hex_thous_hunds, uint8_t hex_tens_ones, uint8_t hex_fraction, uint8_t hex_ex_prec)
{

  double thousHundsResult = static_cast<double>(hex_thous_hunds);
  double tensOnesResult = static_cast<double>(hex_tens_ones);
  double fractionResult = static_cast<double>(hex_fraction);
  double exPrecResult = static_cast<double>(hex_ex_prec);

  return thousHundsResult * 100 + tensOnesResult + fractionResult * (0.01) + exPrecResult * 0.0001;
}

// activate_SOL("1", 1, 1); turns on board 1 sol 1
void command2pin(String solboardIDnum, char command, char mode)
{
  twai_message_t txMessage_command;
  int ID = solboardIDnum.toInt();
  txMessage_command.identifier = ID * 0x10 + 0x0F; // Solenoid Board WRITE COMMAND 0xIDf
  txMessage_command.flags = TWAI_MSG_FLAG_EXTD;    // Example flags (extended frame)
  txMessage_command.data_length_code = 8;          // Example data length (8 bytes)
  txMessage_command.data[0] = 0xFF;                // Sol 0 //Be default, over CAN hex FF does not trigger valid response
  txMessage_command.data[1] = 0xFF;                // Sol 1 //HOWEVER, String FXXX over serial may update frequency
  txMessage_command.data[2] = 0xFF;                // Sol 2
  txMessage_command.data[3] = 0xFF;                // Sol 3
  txMessage_command.data[4] = 0xFF;                // Sol 4
  txMessage_command.data[5] = 0xFF;                // NIL
  txMessage_command.data[6] = 0xFF;                // NIL
  txMessage_command.data[7] = 0xFF;                // NIL
  int statusPin;
  if (command == '0')
  {
    statusPin = 0;
  }
  else if (command == '1')
  {
    statusPin = 1;
  }
  else if (command == '2')
  {
    statusPin = 2;
  }
  else if (command == '3')
  {
    statusPin = 3;
  }
  else if (command == '4')
  {
    statusPin = 4;
  }
  // xSemaphoreTake(mutex_d, portMAX_DELAY);
  if (mode == '0')
  {
    pinStatus[ID][statusPin] = 0;
    txMessage_command.data[statusPin] = 0;
  }
  else if (mode == '1')
  {
    pinStatus[ID][statusPin] = 1;
    txMessage_command.data[statusPin] = 1;
  }
  else
  {
    Serial.println("Invalid mode");
  }
  twai_transmit(&txMessage_command, pdMS_TO_TICKS(1));
  // xSemaphoreGive(mutex_d);
  Serial.println("CAN sent");
}

// RTOS Tasks, not working as of 3/8 12:31 AM - Matthew
void SixDOF_Task(void *pvParameters)
{
  for (;;)
  {
    String accel = _6DOF.printSensorData();
    Serial.println(accel);
    vTaskDelay(pdMS_TO_TICKS(200));
    vTaskDelete(NULL);
  }
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
void PHT_Task(void *pvParameters)
{
  double altitude = Alt.getAltitude();
  Serial.println(String(altitude));
}
void Test_Task(void *pvParameters)
{
  for (;;)
  {
  }
}
void allSol()
{
  delay(100);
  command2pin("01", '1', '1');
  delay(100);
  command2pin("01", '2', '1');
  // command2pin("01", '2', '1');
  //  delay(100);
  //   command2pin("01", '1', '0');
  //  command2pin("01", '2', '1');
  //  delay(100);
  //   command2pin("01", '2', '0');
  //  command2pin("01", '3', '1');
  //  delay(100);
  //   command2pin("01", '3', '0');
}
double calculateRateOfChangeTest(double AltArray[], int READINGS_LENGTH)
{
  double currentMedian = returnMedianTest(AltArray, READINGS_LENGTH);

  double rate_of_change = currentMedian - previousMedianTest;

  previousMedianTest = currentMedian;

  return rate_of_change;
}

// RX Code

int altCount = 0;
int rocCount = 0;
void BabyStateMachine(SixDOF &_6DOF1, PHT &_Alt, GPS &_GPS1, double bottomAlt, double bottomLat, double bottomLong)
// Smaller scale, incomplete state machine used for testing.
//
{
  // Serial.println("Entered state machine");
  // Run an initial Senesor Check, verify
  String userInput = "";
  switch (current_state1)
  {
  case LaunchState::PreIgnition:
  {
    // Serial.println("Entered preignition state");
    // Run an initial Senesor Check, verify
    Serial.println(String(_6DOF1.getNetAccel()));
    double PHT_alt = _Alt.getAltitude();
    double GPS_alt = _GPS1.getAltitude();
    bool PHT1_error = (PHT_alt == 0);
    bool GPS1_error = !(GPS1.fix);
    double final_alt = 0;
    // if (!(PHT1_error && GPS1_error))
    // {
    //   command2pin("02", '3', '1');
    // }

    if (!PHT1_error && !GPS1_error)
    {
      final_alt = (PHT_alt + GPS_alt) / 2;
    }
    else if (!PHT1_error)
    {
      final_alt = PHT_alt;
    }
    else if (!GPS1_error)
    {
      final_alt = GPS_alt;
    }

    startingAltitude = final_alt;
    offset = expectedAltitude - startingAltitude;

    AltArrayTest[altCount % 10] = final_alt;
    altCount = (altCount + 1) % 10;

    char c = Serial.read();
    if (c == '\n') // if Enter pressed,
    {
      if (userInput == "ARM")
      {
        Serial.println("ROCKET IS ARMED AND ON THE PAD. "); // MAYBE WE SHOULD HAVE AN AUDIBLE OR SOME PHYSICAL RESPONSE
        delay(5000);
        // Just for testing, each state will correspond to a solenoid.
        // command2pin("01", '0', '1') PREIGNITION STATE PASSED
        current_state1 = LaunchState::Ignition_to_Apogee;
      }
    }
    else
    {
      userInput += c;
    }
    if (_6DOF1.getNetAccel() >= 15)
    { // change to match flight accel
      preignitionCount += 1;
      Serial.println("PreignitionAccelReached");
    }
    if (preignitionCount > 4)
    {
      // allSol();
      command2pin("01", '0', '1');
      // delay(100);
      // command2pin("01", '1', '0');
      // delay(2000);
      // delay(100);
      // command2pin("01", '2', '0');
      Serial.println("Large acceleration experienced ");
      delay(10000);
      current_state1 = LaunchState::Ignition_to_Apogee;
    }
    // due to low priority of sensor readings, occasionally check every 8 seconds.
    // Save and Print GPS COORD;
    // Save and Print PHT Height;
    // Save and Print Acceleration;
    // Send CAN MSG, Compare with StateMachine2?
    break;
  }
  case LaunchState::Ignition_to_Apogee:
  {
    static int count = 0;

    // Read sensor values
    double PHT_alt = _Alt.getAltitude();
    double GPS_alt = _GPS1.getAltitude();
    double bottom_alt = bottomAlt; // Using the provided bottom altitude

    // Error detection
    bool PHT_error = (PHT_alt == 0);
    // Serial.println("PHT error: " + String(PHT_error));
    bool GPS_error = !(_GPS1.fix);
    // Serial.println("GPS error: " + String(GPS_error));
    bool bottom_error = (bottom_alt == 0);
    // Serial.println("Bottom Sensors Error: " + String(bottom_error));

    // Calculate final altitude with weighted agreement between sensors
    double final_altitude = 0;
    int agreement_count = 0;

    // Check for agreement between PHT and GPS (primary sensors)
    bool PHT_GPS_agree = (!PHT_error && !GPS_error && fabs(PHT_alt - GPS_alt) < 0.5);

    // Check for agreement between PHT and bottom alt
    bool PHT_bottom_agree = (!PHT_error && !bottom_error && fabs(PHT_alt - bottom_alt) < 0.5);

    // Check for agreement between GPS and bottom alt
    bool GPS_bottom_agree = (!GPS_error && !bottom_error && fabs(GPS_alt - bottom_alt) < 0.5);

    // Priority 1: At least two sensors agree (with preference for PHT-GPS agreement)
    if (PHT_GPS_agree)
    {
      // Strong preference for PHT-GPS agreement
      final_altitude = (PHT_alt * 0.6 + GPS_alt * 0.4); // Weight PHT slightly more
      agreement_count = 2;
    }
    else if (PHT_bottom_agree || GPS_bottom_agree)
    {
      // Secondary preference for other agreements
      if (PHT_bottom_agree && GPS_bottom_agree)
      {
        // All three agree (within margins)
        final_altitude = (PHT_alt + GPS_alt + bottom_alt) / 3.0;
        agreement_count = 3;
      }
      else if (PHT_bottom_agree)
      {
        final_altitude = (PHT_alt * 0.5 + bottom_alt * 0.5); // Weight PHT more
        agreement_count = 2;
      }
      else
      {                                                      // GPS_bottom_agree
        final_altitude = (GPS_alt * 0.5 + bottom_alt * 0.5); // Weight GPS more
        agreement_count = 2;
      }
    }
    // Priority 2: Use available sensors with preference order
    else
    {
      // Prefer PHT if available
      if (!PHT_error)
      {
        final_altitude = PHT_alt;
        // If GPS is also available but doesn't agree, we might want to know
        if (!GPS_error)
        {
          // Large discrepancy - might want to flag this
          if (fabs(PHT_alt - GPS_alt) > 2.0)
          {
            Serial.println("Warning: Large PHT-GPS altitude discrepancy");
          }
        }
      }
      // Fall back to GPS if PHT not available
      else if (!GPS_error)
      {
        final_altitude = GPS_alt;
      }
      // Final fallback to bottom altitude
      else if (!bottom_error)
      {
        final_altitude = bottom_alt;
      }
      else
      {
        // All sensors failed - this should be handled as an error
        Serial.println("Error: All altitude sensors failed!");
        // Might want to implement recovery logic here
        final_altitude = 0; // Or maintain last known good value
      }
    }

    // Store the altitude values for rate of change calculation
    Serial.println("Final Altitude = " + String(final_altitude));
    Serial.println("Count = " + String(count));
    AltArrayTest[count % 10] = final_altitude;
    count++;

    // Calculate rate of change and check for apogee
    if (count >= 10)
    {
      double rate_of_change = calculateRateOfChangeTest(AltArrayTest, 10);
      Serial.println("Rate of change: " + String(rate_of_change));

      // More conservative apogee detection with additional checks
      if ((rate_of_change < 0))
      {
        rocCount++;

        // Additional verification: check if altitude is actually decreasing
        // Confirm with current altitude vs previous
        // double current_alt = final_altitude;
        // double prev_alt = AltArrayTest[(count - 3) % 10]; // Value from 3 readings ago

        // if (current_alt < prev_alt)
        // {
        if (rocCount > 2)
        {
          Serial.println("Apogee detected - transitioning to descent phase");
          command2pin("01", '1', '1');
          delay(10000);
          current_state1 = LaunchState::_1000ft;
        }

        // count = 0;
        // }
      }
      else
      {
        rocCount = 0;
      }

      // Optional: Reset count periodically to use fresh data
      // if (count >= 20)
      //   count = 0;
    }

    // delay(100); // Adjust delay as needed for sensor updates
    break;
  }
  case LaunchState::_1000ft:
  {
    // Read sensor values
    double PHT_alt = _Alt.getAltitude();
    double GPS_alt = _GPS1.getAltitude();
    double bottom_alt = bottomAlt;

    // Error detection
    bool PHT_error = (PHT_alt == 0);
    bool GPS_error = !(_GPS1.fix);
    bool bottom_error = (bottom_alt == 0);

    // Calculate final altitude (weighted if multiple sensors available)
    double final_altitude = 0;

    if (!PHT_error && !GPS_error)
    {
      final_altitude = (PHT_alt + GPS_alt) / 2; // Average if both work
    }
    else if (!PHT_error)
    {
      final_altitude = PHT_alt; // Fallback to PHT
    }
    else if (!GPS_error)
    {
      final_altitude = GPS_alt; // Fallback to GPS
    }
    else if (!bottom_error)
    {
      final_altitude = bottom_alt; // Last resort: bottom sensor
    }
    else
    {
      break; // All sensors failed; stay in current state
    }

    Serial.println("Current Altitude: " + String(final_altitude) + " ft");

    // Transition when below 1000 ft
    if (final_altitude < 10 + startingAltitude)
    {
      Serial.println("1000 ft threshold reached! Moving to 900 ft state.");
      command2pin("01", '2', '1'); // Trigger solenoid/parachute
      delay(10000);
      current_state1 = LaunchState::_900ft;
    }
    break;
  }
  case LaunchState::_900ft:
  {
    double PHT_alt = _Alt.getAltitude();
    double GPS_alt = _GPS1.getAltitude();
    double bottom_alt = bottomAlt;

    bool PHT_error = (PHT_alt == 0);
    bool GPS_error = !(_GPS1.fix);
    bool bottom_error = (bottom_alt == 0);

    double final_altitude = 0;

    if (!PHT_error && !GPS_error)
    {
      final_altitude = (PHT_alt + GPS_alt) / 2;
    }
    else if (!PHT_error)
    {
      final_altitude = PHT_alt;
    }
    else if (!GPS_error)
    {
      final_altitude = GPS_alt;
    }
    else if (!bottom_error)
    {
      final_altitude = bottom_alt;
    }
    else
    {
      break;
    }

    Serial.println("Current Altitude: " + String(final_altitude) + " ft");

    // Transition when below 900 ft
    if (final_altitude < 5 + startingAltitude)
    {
      Serial.println("900 ft threshold reached! Moving to 800 ft state.");
      command2pin("01", '3', '1'); // Next parachute action
      delay(10000);
      current_state1 = LaunchState::_800ft;
    }
    break;
  }
  case LaunchState::_800ft:
  {
    double PHT_alt = _Alt.getAltitude();
    double GPS_alt = _GPS1.getAltitude();
    double bottom_alt = bottomAlt;

    bool PHT_error = (PHT_alt == 0);
    bool GPS_error = !(_GPS1.fix);
    bool bottom_error = (bottom_alt == 0);

    double final_altitude = 0;

    if (!PHT_error && !GPS_error)
    {
      final_altitude = (PHT_alt + GPS_alt) / 2;
    }
    else if (!PHT_error)
    {
      final_altitude = PHT_alt;
    }
    else if (!GPS_error)
    {
      final_altitude = GPS_alt;
    }
    else if (!bottom_error)
    {
      final_altitude = bottom_alt;
    }
    else
    {
      break;
    }

    Serial.println("Current Altitude: " + String(final_altitude) + " ft");

    // Transition when below 800 ft
    if (final_altitude < 800)
    {
      Serial.println("800 ft threshold reached! Moving to descent phase.");
      // command2pin("06", '2', '1'); // Final parachute action
      current_state1 = LaunchState::Descent;
    }
    break;
  }
  case LaunchState::Descent:
  {
    // command2pin("06", '3', '1');
    delay(5000);
    current_state1 = LaunchState::Touchdown;
    break;
  }
  case LaunchState::Touchdown:
  {

    break;
  }
  }
}

uint8_t extraPrecision(double dataValue)
{
  int integerPart = static_cast<int>(dataValue);
  double decimalPart = dataValue - integerPart;
  int scaledDecimal = static_cast<int>(decimalPart * 10000);
  int filteredDecimal = scaledDecimal % 100;
  // if (scaledDecimal > 255)
  // {
  //     scaledDecimal = 255;
  // }
  uint8_t hexValue = 0x00 + static_cast<uint8_t>(filteredDecimal);
  return hexValue;
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

// TWAI/CAN RECIEVE MESSAGE
void commandTask(String can_code)
{
  // String canMessage = *(String *)pvParameters;  // Cast and dereference the passed parameter for FreeRTOS specific format

  while (1)
  {
    // Using the passed message
    String message = can_code;

    // Extract solboardIDnum, command, and mode from the message
    String solboardIDnum = message.substring(0, message.length() - 2);
    char command = message[message.length() - 2];
    char mode = message[message.length() - 1];

    switch (command)
    {
    case '0':
    case '1':
    case '2':
    case '3':
    case '4':
      command2pin(solboardIDnum, command, mode); // sends command to activate the solenoid HIGH
      break;
    default:
      vTaskDelay(10 / portTICK_PERIOD_MS); // Delay for a while
      yield();
      vTaskDelay(10);
    }
  }
}
void printTwaiStatus()
{
  twai_status_info_t status;
  twai_get_status_info(&status);
  Serial.print("Status:  ");
  switch (status.state)
  {
  case (TWAI_STATE_STOPPED):
    Serial.println("Stopped");
    break;
  case (TWAI_STATE_RUNNING):
    Serial.println("Running");
    break;
  case (TWAI_STATE_BUS_OFF):
    Serial.println("Bus Off");
    break;
  case (TWAI_STATE_RECOVERING):
    Serial.println("Recovering");
    break;
  default:
    Serial.println("Unknown");
    break;
  }
  Serial.print("Tx Error Counter: ");
  Serial.println(status.tx_error_counter);
  Serial.print("Rx Error Counter: ");
  Serial.println(status.rx_error_counter);
  Serial.print("Bus Error Count: ");
  Serial.println(status.bus_error_count);
  Serial.print("Messages to Tx: ");
  Serial.println(status.msgs_to_tx);
  Serial.print("Messages to Rx: ");
  Serial.println(status.msgs_to_rx);
}

void setup()
{
  Serial.begin(921600); // Initialize Serial communication
  Serial.println("Begin");
  while (!Serial)
  {
    Serial.print("Serial Failed to start");
    delay(10);
  }
  // CAN Setup:
  pinMode(CAN_TX, OUTPUT);
  pinMode(CAN_RX, INPUT);
  // Config CAN Speed
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX, (gpio_num_t)CAN_RX, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS(); // TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  // Install and start the TWAI driver
  esp_err_t canStatus = twai_driver_install(&g_config, &t_config, &f_config);
  if (canStatus == ESP_OK)
  {
    Serial.println("CAN Driver installed");
  }
  else
  {
    Serial.println("CAN Driver installation failed");
  }
  if (twai_start() != ESP_OK)
  {
    Serial.println("Error starting TWAI!");
  }
  // starts TWAI
  Serial.println("CAN/TWAI BUS STARTED");
  ESP32Can.setRxQueueSize(8);
  ESP32Can.setTxQueueSize(8);
  // Create and assign tasks for each core
  // xTaskCreatePinnedToCore(SixDOF_Task, "Six_DOF_Task", 2048, NULL, 1, NULL, 0);
  // xTaskCreatePinnedToCore(PHT_Task, "PHT_Task", 2048, NULL, 2, NULL, 1); // PHT broke af rn
  // xTaskCreatePinnedToCore(Test_Task, "Print", 2048, NULL, 2, NULL, 1);

  if (!(_6DOF.start_6DOF()))
  {
    Serial.println("6DOF Failed to start");
  }
  I2C1.begin(42, 41);
  if (!Alt.connectSensor())
  {
    Serial.println("Error connecting to PHT sensor");
  }
  else
  {
    Serial.println("Connected to Alt sensor");
    Alt.setSensorConfig();
  }
  GPS1.startGPS();
  // if(!Alt.startPHT()){
  //   Serial.println("Altimeter Failed to start");
  // }
  // Serial.println("Altimeter Started");       CHANGE ALTIMETER TO MSx07
  // command2pin("01", '0', '0');
  // command2pin("01", '1', '0');
  // command2pin("01", '2', '0');
  // command2pin("01", '3', '0');

  // command2pin("02", '0', '0');
  // command2pin("02", '1', '0');
  // command2pin("02", '2', '0');
  // command2pin("02", '3', '0');

  delay(300);
}

void loop()
{
  double bottomAltitude;
  double bottomLongitude;
  double bottomLatitude;
  if (ESP32Can.readFrame(rxFrame, 1000))
  { // Wait for frame with 1 second timeout
    // printTwaiStatus();
    // Serial.printf("Received frame: ID=0x%03X, Length=%d\n", rxFrame.identifier, rxFrame.data_length_code);

    if (rxFrame.identifier == 0x04)
    { // GPS Frame
      if (rxFrame.data_length_code == 8)
      {
        // Parse GPS data
        bottomLatitude = hextoDecimal(rxFrame.data[0], rxFrame.data[1],
                                      rxFrame.data[2], rxFrame.data[3]);
        bottomLongitude = hextoDecimal(rxFrame.data[4], rxFrame.data[5],
                                       rxFrame.data[6], rxFrame.data[7]);

        // Serial.print("GPS Coordinates - Latitude: ");
        // Serial.print(bottomLatitude, 6);
        // Serial.print(", Longitude: ");
        // Serial.println(bottomLongitude, 6);
      }
      else if (rxFrame.data_length_code == 6)
      {
        for (int i = 0; i < 3; i++)
        {
          bottomAltitude = hextoDecimal(0, rxFrame.data[i * 2],
                                        rxFrame.data[i * 2 + 1], 0);
          // Serial.print("Altitude Reading ");
          // Serial.print(i + 1);
          // Serial.print(": ");
          // Serial.println(String(bottomAltitude));
        }
      }
      else
      {
        // Serial.println("Invalid frame length (expected 8)");
      }
    }
    else
    {
      // Serial.println("Unknown frame ID");
    }
  }
  else
  {
    // printTwaiStatus();
    // Serial.println("No Frame read yet");
  }
  unsigned long past_time = millis();

  // each pin works individually, but cannot actuate multiple at once. maybe a MSG priority issue
  BabyStateMachine(_6DOF, Alt, GPS1, bottomAltitude, bottomLatitude, bottomLongitude);
  // command2pin("07", '2', '1');
  // command2pin("06", '2', '1');
  // delay(100);
  // command2pin("07", '3', '1');
  // command2pin("06", '3', '1');
  // delay(100);
  // command2pin("07", '0', '1');
  // command2pin("06", '0', '1');
  // delay(100);
  // command2pin("07", '1', '1');
  // command2pin("06", '1', '1');
  // delay(100);
  // // delay(500);
  // // command2pin("01", '2', '1');
  // // delay(500);
  // // command2pin("01", '3', '0');
  // // delay(1000);
  // command2pin("07", '2', '0');
  // command2pin("06", '2', '0');
  // delay(100);
  // // delay(500);
  // command2pin("07", '3', '0');
  // command2pin("06", '3', '0');
  // delay(100);
  // command2pin("07", '0', '0');
  // command2pin("06", '0', '0');
  // delay(100);
  // // delay(500);
  // command2pin("07", '1', '0');
  // command2pin("06", '1', '0');
  // delay(100);

  // double pressure = Alt.getPressure();
  // double altitude = Alt.getAltitude();
  // Serial.println("Pressure: " + String(pressure) + ", Altitude: " + String(altitude));
  // Serial.println(String(altitude));
  // String accel = _6DOF.printSensorData();
  // Serial.println(accel);
  // Serial.println("Loop Time (ms): "+String(millis()-past_time)); // loop time
  // allSol();
  // printTwaiStatus();
  // delay(100);
  // Serial.println("End of Loop");
}
