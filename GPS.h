#ifndef _GPS_H
#include <Adafruit_GPS.h>


#define GPSECHO  true
#define TX_PIN 17
#define RX_PIN 25
#define GPSSerial Serial1
class GPS : public Adafruit_GPS {
  public:
    GPS();
    void startGPS();
    void printInfo();
};
#endif
