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
#include <ArduinoJson.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/twai.h"
#include <task.h>
#include <queue.h>
// #include "LaunchState.h"
#define SEALEVELPRESSURE_HPA (1013.25)
#define LSM_CS 5
#define LSM_SCK 18
#define LSM_MISO 19
#define LSM_MOSI 23
SixDOF _6DOF;
// PHT Alt;
NineDOF mpu;
GPS GPS1;
// TwoWire I2C_n(1);
TwoWire I2C_one(0);
PHT Alt(I2C_one);
uint16_t measurement_delay_us = 65535; // Delay between measurements for testing
// LaunchState Halya;
NineDOF _9DOF;

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
int canTXRXcount[2] = {0, 0};

void transmitTask(void *pvParameters);
void receiveTask(void *pvParameters);
void commandTask(void *pvParameters);
String messageToCAN(String code);
// Define CAN pins
#define CAN_TX 16
#define CAN_RX 17

int groundLevelAltitudeTest = 0;
// std::map<string, bool> statesMapTest;
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

void command2pin(String solboardIDnum, char solIndex, char mode)
{
    twai_message_t txMessage_command;
    // Convert solboardIDnum to integer, ensuring it is valid
    int ID = solboardIDnum.toInt();
    if (ID < 0 || ID > 255)
    { // Limit range to prevent errors
        Serial.println("Invalid solboard ID");
        return;
    }
    Serial.print("Solboard ID: ");
    Serial.println(ID);
    txMessage_command.identifier = ID * 0x10 + 0x0F; // Solenoid Board WRITE COMMAND 0xIDF
    txMessage_command.flags = TWAI_MSG_FLAG_EXTD;    // Extended frame format
    txMessage_command.data_length_code = 8;          // 8-byte message
    // Initialize all solenoid states to 0
    memset(txMessage_command.data, 0, sizeof(txMessage_command.data));
    // Determine solenoid index
    int statusPin = solIndex - '0'; // Convert char ('0'-'3') to int (0-3)
    if (statusPin < 0 || statusPin > 3)
    {
        Serial.println("Invalid solenoid index");
        return;
    }
    // Set the requested solenoid state
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
        return;
    }
    // Transmit the CAN message
    if (twai_transmit(&txMessage_command, pdMS_TO_TICKS(1)) == ESP_OK)
    {
        Serial.println("Solenoid " + String(solIndex) + " actuated successfully");
    }
    else
    {
        Serial.println("Solenoid " + String(solIndex) + " actuation failed");
    }
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

    GPS1.startGPS();
    delay(500);

    // if (!(_6DOF.start_6DOF()))
    // {
    //   Serial.println("6DOF Failed to start");
    // }
    // Altimeter
    // Alt.startPHT();
    I2C_one.begin(42, 41);
    if (!Alt.connectSensor())
    {
        Serial.println("Error connecting to Alt sensor...");
    }
    else
    {
        Serial.println("Connected to Alt sensor");
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

// static int counter = 0;

void loop()
{
    // static unsigned long start_time = millis();
    // if (millis() - start_time < 5000)
    // {
    //   return; // Wait for 5 seconds before proceeding
    // }
    unsigned long past_time = millis();
    // each pin works individually, but cannot actuate multiple at once. maybe a MSG priority issue
    // BabyStateMachine(_6DOF,Alt,mpu,GPS1); //is having 8 sensors fed into the state machine the best way to do? How would we do this running on just one sensor board?
    // String accel = _6DOF.printSensorData();
    // Serial.println(accel);
    // Serial.println(String(millis() - past_time)); // loop time
    // command2pin("01", '0', '0');
    // command2pin("01", '1', '0');
    // command2pin("01", '2', '0');
    // command2pin("01", '3', '0');

    static int count = 0;

    // Read sensor values
    double PHT_alt = Alt.getAltitude();
    double GPS_alt = GPS1.getAltitude();

    // xSerial.println("PHT altitude reading: " + String(PHT_alt) + ", GPS altitude reading: " + String(GPS_alt));

    // Error detection
    bool PHT_error = (PHT_alt == 0);
    bool GPS_error = !(GPS1.fix);

    // Serial.println("PHT error: " + String(PHT_error) + ", GPS_error: " + String(GPS_error));

    double final_altitude = 0;

    if (!PHT_error && !GPS_error)
    {
        final_altitude = (PHT_alt + GPS_alt) / 2.0; // Average both sensors
    }
    else if (!PHT_error)
    {
        final_altitude = PHT_alt;
    }
    else if (!GPS_error)
    {
        final_altitude = GPS_alt;
    }

    // Store altitude values for rate of change calculation
    AltArrayTest[count % 10] = final_altitude;
    count++;

    // Step 5: Check for apogee based on rate of change
    if (count >= 10)
    {
        double rate_of_change = calculateRateOfChangeTest(AltArrayTest, 10);
        Serial.println("Rate of Change: " + String(rate_of_change));

        if ((fabs(rate_of_change) < 0.1 || rate_of_change < 0))
        {
            Serial.println("Halya has reached apogee!");
            // command2pin("01", '0', '1');
            // command2pin("01", '2', '1');
        }
        count = 0;
    }

    delay(100); // Delay for sensor updates
}
