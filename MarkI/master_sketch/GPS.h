#ifndef GPS_H
#define GPS_H

#include <Arduino.h>
#include <SoftwareSerial.h>

struct GPSInfo {
    char time[10];
    char lat[11];
    char lon[12];
    long alt;
};

class GPS {

private:
    int baud;
    SoftwareSerial serial;

    bool getAck(unsigned char packetClass, unsigned char packetId);
    void sendUbx(uint8_t *MSG, uint8_t len);
    uint8_t fromHex(char x);

public:
    GPS(int rx, int tx, int baud);
    bool init();
    bool poll(GPSInfo *gpsInfo);

};

#endif /* GPS_H */
