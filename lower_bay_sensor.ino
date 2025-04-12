#include "driver/twai.h"
#include "GPS.h"
#include "PHT.h"
#include "SixDOF.h"
#include "NineDOF.h"
#define CAN_TX 16
#define CAN_RX 17
#define SEALEVELPRESSURE_HPA (1013.25)
#define LSM_CS 38
#define LSM_SCK 37
#define LSM_MISO 35
#define LSM_MOSI 36

#define PHT_SDA 42
#define PHT_SCL 41

#define ICM_SDA 39
#define ICM_SCL 40

TwoWire I2C2(1);  // Default I2C bus
PHT alt_lower(I2C2);
GPS GPS_lower;
SixDOF six_lower;
NineDOF nine_lower;
uint8_t first_two_digits(double dataValue) {
  int integerPart = static_cast<int>(dataValue);
  int first_two = integerPart % 100;
  //  if (integerPart > 255)
  //    {
  //        integerPart = 255;
  //    }
  return 0x00 + static_cast<uint8_t>(first_two);
}

uint8_t last_two_digits(double dataValue) {
  int integerPart = static_cast<int>(dataValue);
  int last_two = integerPart / 100;
  //  if (integerPart > 255)
  //    {
  //        integerPart = 255;
  //    }
  return 0x00 + static_cast<uint8_t>(last_two);
}

uint8_t integerPartToHex(double dataValue) {
  int integerPart = static_cast<int>(dataValue);
  if (integerPart > 255) {
    integerPart = 255;
  }
  return 0x00 + static_cast<uint8_t>(integerPart);
}
uint8_t decimalPartToHex(double dataValue) {
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
uint8_t extraPrecision(double dataValue) {
  Serial.print("Extra precision: ");
  Serial.println(dataValue, 4);
  int integerPart = static_cast<int>(dataValue);
  double decimalPart = dataValue - integerPart;
  Serial.print("Decimal part: ");
  Serial.println(decimalPart, 4);
  int scaledDecimal = static_cast<int>(decimalPart * 10000);
  Serial.print("Scaled decimal part: ");
  Serial.println(scaledDecimal, 4);
  int filteredDecimal = scaledDecimal % 100;
  Serial.print("Filtered decimal part: ");
  Serial.println(filteredDecimal);
  if (scaledDecimal > 255) {
    scaledDecimal = 255;
  }
  uint8_t hexValue = 0x00 + static_cast<uint8_t>(filteredDecimal);
  return hexValue;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  I2C2.begin(42, 41);
  if (!alt_lower.connectSensor()) {
    Serial.println("PHT failed to connect");
  } else {
    Serial.println("PHT connected");
  }
  //GPS_lower.startGPS();
  if (!six_lower.start_6DOF()) {
    Serial.println("Six DOF failed to connect");
  } else {
    Serial.println("Six DOF connected");
  }
  nine_lower.begin();
  
  pinMode(CAN_TX, OUTPUT);
  pinMode(CAN_RX, INPUT);

  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX, (gpio_num_t)CAN_RX, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();  //TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  esp_err_t canStatus = twai_driver_install(&g_config, &t_config, &f_config);
  if (canStatus == ESP_OK) {
    Serial.println("CAN Driver installed");
  } else {
    Serial.println("CAN Driver installation failed");
  }
  twai_start();

  Serial.println("CAN Started");
}

void loop() {
  // put your main code here, to run repeatedly:
  //GPS Transmission
  static uint32_t lastStamp = 0;
  uint32_t currentStamp = millis();
  if (currentStamp - lastStamp > 10) {  // Send message every 0.1 seconds
    // PHT
    lastStamp = currentStamp;
    //      // Returns number of messages in queue
    //      //Serial.println(ESP32Can.inTxQueue());
    twai_message_t txFrame;
    txFrame.identifier = 0x04;  // upper bay board id
    txFrame.extd = 0;
    txFrame.data_length_code = 8;
    nmea_float_t GPS_latitude = GPS_lower.getLatitude();
    nmea_float_t GPS_longitude = GPS_lower.getLongitude();
    double GPS_latitude_degrees = GPS_lower.convertToDegrees(GPS_latitude);
    Serial.print(GPS_latitude_degrees);
    txFrame.data[0] = first_two_digits(GPS_latitude_degrees);
    txFrame.data[1] = last_two_digits(GPS_latitude_degrees);
    txFrame.data[2] = decimalPartToHex(GPS_latitude_degrees);
    txFrame.data[3] = extraPrecision(GPS_latitude_degrees);
    double GPS_longitude_degrees = GPS_lower.convertToDegrees(GPS_longitude);
    Serial.print(GPS_longitude_degrees);
    txFrame.data[4] = first_two_digits(GPS_longitude_degrees);
    txFrame.data[5] = last_two_digits(GPS_longitude_degrees);
    txFrame.data[6] = decimalPartToHex(GPS_longitude_degrees);  // Best to use 0xAA (0b10101010) instead of 0
    txFrame.data[7] = extraPrecision(GPS_longitude_degrees);
    //       ... fill other data bytes if needed
    esp_err_t gps_result = twai_transmit(&txFrame, pdMS_TO_TICKS(100));  // 100ms timeout

    if (gps_result == ESP_OK) {
      Serial.println("GPS Transmission successful!");
    } else {
      Serial.print("GPS Transmission failed! Error code: ");
      Serial.println(gps_result);
    }
  }
  twai_message_t txFrame;
  double altitude_reading = alt_lower.getAltitude();
  txFrame.identifier = 0x04;     // Different CAN ID for second frame
  txFrame.extd = 0;              // Standard frame
  txFrame.data_length_code = 6;  // Data length
  altitude_reading = alt_lower.getAltitude();
  Serial.println(altitude_reading);
  txFrame.data[0] = integerPartToHex(altitude_reading);
  txFrame.data[1] = decimalPartToHex(altitude_reading);
  altitude_reading = alt_lower.getAltitude();
  Serial.println(altitude_reading);
  txFrame.data[2] = integerPartToHex(altitude_reading);
  txFrame.data[3] = decimalPartToHex(altitude_reading);  // Best to use 0xAA (0b10101010) instead of 0
  altitude_reading = alt_lower.getAltitude();
  Serial.println(altitude_reading);
  txFrame.data[4] = integerPartToHex(altitude_reading);                // CAN works better this way as it needs
  txFrame.data[5] = decimalPartToHex(altitude_reading);                // to avoid bit-stuffing
  esp_err_t pht_result = twai_transmit(&txFrame, pdMS_TO_TICKS(100));  // 100ms timeout
  if (pht_result == ESP_OK) {
    Serial.println("PHT frame transmitted successfully!");
  } else {
    Serial.println("Failed to transmit PHT frame.");
  }

  // double six_dof_net = six_lower.getNetAccel();
  // vector<double> six_accelArr = six_lower.getAcceleration();
  // Serial.println(six_dof_net);
  // txFrame.identifier = 0x04;     // Different CAN ID for second frame
  // txFrame.extd = 0;              // Standard frame
  // txFrame.data_length_code = 8;  // Data length
  // txFrame.data[0] = integerPartToHex(six_accelArr[0]);
  // txFrame.data[1] = decimalPartToHex(six_accelArr[0]);
  // txFrame.data[2] = integerPartToHex(six_accelArr[1]);
  // txFrame.data[3] = decimalPartToHex(six_accelArr[1]);  // Best to use 0xAA (0b10101010) instead of 0
  // txFrame.data[4] = integerPartToHex(six_accelArr[2]);  // CAN works better this way as it needs
  // txFrame.data[5] = decimalPartToHex(six_accelArr[2]);  // to avoid bit-stuffing
  // txFrame.data[6] = integerPartToHex(six_dof_net);
  // txFrame.data[7] = decimalPartToHex(six_dof_net);
  // esp_err_t sixdof_result = twai_transmit(&txFrame, pdMS_TO_TICKS(100));  // 100ms timeout
  // if (sixdof_result == ESP_OK) {
  //   Serial.println("Six DOF frame transmitted successfully!");
  // } else {
  //   Serial.println("Failed to transmit Six DOF frame.");
  // }

  // double nine_dof_net = nine_lower.getNetAccel();
  // vector<double> nine_accelArr = six_lower.getAcceleration();
  // Serial.println(nine_dof_net);
  // txFrame.identifier = 0x04;     // Different CAN ID for second frame
  // txFrame.extd = 0;              // Standard frame
  // txFrame.data_length_code = 8;  // Data length
  // txFrame.data[0] = integerPartToHex(nine_accelArr[0]);
  // txFrame.data[1] = decimalPartToHex(nine_accelArr[0]);
  // txFrame.data[2] = integerPartToHex(nine_accelArr[1]);
  // txFrame.data[3] = decimalPartToHex(nine_accelArr[1]);  // Best to use 0xAA (0b10101010) instead of 0
  // txFrame.data[4] = integerPartToHex(nine_accelArr[2]);  // CAN works better this way as it needs
  // txFrame.data[5] = decimalPartToHex(nine_accelArr[2]);  // to avoid bit-stuffing
  // txFrame.data[6] = integerPartToHex(nine_dof_net);
  // txFrame.data[7] = decimalPartToHex(nine_dof_net);
  // esp_err_t ninedof_result = twai_transmit(&txFrame, pdMS_TO_TICKS(100));  // 100ms timeout
  // if (ninedof_result == ESP_OK) {
  //   Serial.println("Nine DOF frame transmitted successfully!");
  // } else {
  //   Serial.println("Failed to transmit Nine DOF frame.");
  // }
}
