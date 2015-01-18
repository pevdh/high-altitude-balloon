#include "GPS.h"
#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Print.h>
#include <string.h>

GPS::GPS(int rx, int tx, int baud)
:
serial(rx, tx)
{
    this->baud = baud;
}

bool GPS::init() {
    unsigned long startTime = millis();
    unsigned short timeout = 20000;

    uint8_t setNav[] = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC};
    uint8_t len = sizeof(setNav) / sizeof(uint8_t);
    
    serial.begin(baud);
    while (1) {
        if (millis() - startTime > timeout) return false;

        for(uint8_t i = 0; i < len; i++) {
            serial.write(setNav[i]);
        }
        serial.println();

        if (getAck(setNav[2], setNav[3])) break;
    }

    serial.println(F("$PUBX,40,GLL,0,0,0,0*5C"));
    serial.println(F("$PUBX,40,GGA,0,0,0,0*5A"));
    serial.println(F("$PUBX,40,GSA,0,0,0,0*4E"));
    serial.println(F("$PUBX,40,RMC,0,0,0,0*47"));
    serial.println(F("$PUBX,40,GSV,0,0,0,0*59"));
    serial.println(F("$PUBX,40,VTG,0,0,0,0*5E"));
    serial.end();
    
    serial.end();
    
    return true;
}

void GPS::sendUbx(uint8_t *MSG, uint8_t len) {
  for(int i=0; i<len; i++) {
    Serial.write(MSG[i]);
  }
  //Serial.println();
}

bool GPS::poll(GPSInfo *gpsInfo) {
    unsigned long startTime = millis();
    unsigned short timeout = 20000;

    serial.begin(baud);

    serial.println(F("$PUBX,00*33"));

    char *buff = new char[128];

    char b = 0;
    uint8_t len = 0;

    uint8_t parity = 0;

    uint8_t checksum = 0;
    bool isChecksum = false;
    bool hasChecksumFirstChar = false;
    bool hasChecksumSecondChar = false;

    while (b != '\n') {
        if (millis() - startTime > timeout) {
            free(buff);
            return false;
        }

        if (serial.available()) {
            b = serial.read();

            if (b == '*') {
                isChecksum = true;
            } else if (isChecksum && !hasChecksumFirstChar) {
                checksum += 16 * fromHex(b);
                hasChecksumFirstChar = true;
            } else if (isChecksum && !hasChecksumSecondChar) {
                checksum += fromHex(b);
                hasChecksumSecondChar = true;
            }

            if (!isChecksum && b != '$' && b != '\r' && b != '\n') parity ^= b;

            buff[len++] = b;
        }
    }
    buff[len] = 0x00;
    serial.end();
    
    Serial.print("GPS.poll(): Received NMEA sentence: ");
    Serial.print(buff);
    
    if (parity != checksum) {
        free(buff);
        return false;
    }

    char *info[22];

    uint8_t index = 0;
    uint8_t dataIndex = 0;
    for (b = 0; b < len; b++) {
        if (buff[b] == ',' || buff[b] == '*') {
            dataIndex++;
            buff[b] = 0x00;
            if (dataIndex > 1 && dataIndex < 8) {
                info[index++] = &buff[b] + 1;
            }
        }
    }

    gpsInfo->time[0] = 0x00;
    strcat(gpsInfo->time, info[0]);
    
    char timeBuff[9];
    timeBuff[2] = timeBuff[5] = ':';
    memcpy(timeBuff, gpsInfo->time, 2);
    memcpy(timeBuff + 3, gpsInfo->time + 2, 2);
    memcpy(timeBuff + 6, gpsInfo->time + 4, 2);
    
    memcpy(gpsInfo->time, timeBuff, 9);
    gpsInfo->time[8] = 0x00;
    
    //gpsInfo->time[strlen(gpsInfo->time) - 3] = 0x00; // remove final three chars
    

    gpsInfo->lat[0] = 0x00;
    if (*info[2] == 'S') strcat(gpsInfo->lat, "-");
    strcat(gpsInfo->lat, info[1]);

    gpsInfo->lon[0] = 0x00;
    if (*info[4] == 'W') strcat(gpsInfo->lon, "-");
    strcat(gpsInfo->lon, info[3]);

    for (b = 0; info[5][b] != 0x00; b++) {
        if (info[5][b] == '.') {
            info[5][b] = 0x00;
            //memmove(&info[5][b], &info[5][b] + 1, strlen(info[5]) - b);
            break;
        }
    }

    gpsInfo->alt = strtol(info[5], NULL, 10);

    free(buff);
    return true;
}

uint8_t GPS::fromHex(char a) {
    if (a >= 'A' && a <= 'F')
        return a - 'A' + 10;
    else if (a >= 'a' && a <= 'f')
        return a - 'a' + 10;
    else
        return a - '0';
}

bool GPS::getAck(unsigned char packetClass, unsigned char packetId) {
    uint8_t b;
    uint8_t ackByteID = 0;
    uint8_t ackPacket[10];
    unsigned long startTime = millis();

    ackPacket[0] = 0xB5;	// header
    ackPacket[1] = 0x62;	// header
    ackPacket[2] = 0x05;	// class
    ackPacket[3] = 0x01;	// id
    ackPacket[4] = 0x02;	// length
    ackPacket[5] = 0x00;
    ackPacket[6] = packetClass;	// ACK class
    ackPacket[7] = packetId;	// ACK id
    ackPacket[8] = 0;		// CK_A
    ackPacket[9] = 0;		// CK_B

    // Calculate the checksums
    for (uint8_t i = 2; i < 8; i++) {
        ackPacket[8] = ackPacket[8] + ackPacket[i];
        ackPacket[9] = ackPacket[9] + ackPacket[8];
    }

    while (1) {

        // Test for success
        if (ackByteID > 9) {
            return true;
        }

        // Timeout if no valid response in 3 seconds
        if (millis() - startTime > 3000) {
            return false;
        }

        // Make sure data is available to read
        if (serial.available()) {
            b = serial.read();
            // Check that bytes arrive in sequence as per expected ACK packet
            if (b == ackPacket[ackByteID]) {
                ackByteID++;
            }
            else {
                ackByteID = 0;	// Reset and look again, invalid order
            }

        }
    }
}
