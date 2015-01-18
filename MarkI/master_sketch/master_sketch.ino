#define IS_DEBUG 1
#ifdef IS_DEBUG
  #define LOG_DEBUG(x) logDebug(x)
#else
  #define LOG_DEBUG(x) void(0)
#endif

#include <SoftwareSerial.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>

#include "GPS.h"
#include "RTTY.h"
#include "OneWire.h"
#include "DallasTemperature.h"
#include "HMC5883L.h"

#define USB_SERIAL_BAUD 115200

#define GPS_RX 5
#define GPS_TX 6
#define GPS_BAUD 9600
#define GSM_RX 7
#define GSM_TX 8
#define GSM_POWER 9

#define GSM_BAUD 19200
#define RADIO_TX 3
#define SD_CS 4
#define ONE_WIRE_BUS 2
#define STATUS_LED 10

#define SMS_MAX_HEIGHT 500
#define SMS_INTERVAL 60000

#define VOLTAGE_MEASURE_PIN 0

const char CALLSIGN[] = "$$MEURS";
const char SMS_SEND_CMD[] = "AT+CMGS=\"+00000000000\"\r";
const char FILENAME[] = "DATA.log";

struct Info {
    unsigned int sentenceNumber;
    GPSInfo gps;
    float internalTemp;
    float externalTemp;
    unsigned short heading;
    float batteryLevel;
} payloadInfo;

GPS gps(GPS_RX, GPS_TX, GPS_BAUD);

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DeviceAddress insideThermometer, outsideThermometer;

HMC5883L compass;

SoftwareSerial gsm(GSM_RX, GSM_TX);

long prevAlt = 0;
long prevSmsTime = 0;

void setup() {
  LOG_DEBUG(F("-setup(): Setting up."));
  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, LOW);
  
  #ifdef IS_DEBUG
    Serial.begin(USB_SERIAL_BAUD);
  #endif
  
  blinkLed();
  initSd();
  
  blinkLed();
  initRadio();
  
  blinkLed();
  initGps();
  
  blinkLed();
  initCompass();
  
  blinkLed();
  initTempSensors();
  
  blinkLed();
  initGsm();
  
  payloadInfo = Info();
  payloadInfo.sentenceNumber = 0;
  payloadInfo.gps = GPSInfo();
  
  LOG_DEBUG(F("-setup(): Done & ready for launch."));
  
  digitalWrite(STATUS_LED, HIGH);
}

void loop() {
    LOG_DEBUG(F("-loop(): Updating data"));
    payloadInfo.sentenceNumber++;
    
    sensors.requestTemperatures();
    payloadInfo.internalTemp = sensors.getTempC(insideThermometer);
    payloadInfo.externalTemp = sensors.getTempC(outsideThermometer);
    
    payloadInfo.heading = getHeading();
    payloadInfo.batteryLevel = getBatteryLevel();

    gps.poll(&payloadInfo.gps);
    
    LOG_DEBUG(F("-loop(): Writing new data to sd"));
    
    char* buff = infoToString();
    
    File logFile = openLogFile();
    writeToFile(buff, &logFile);
    logFile.close();
    
    LOG_DEBUG(F("-loop(): Sending data."));
    if (payloadInfo.gps.alt < SMS_MAX_HEIGHT && (millis() - prevSmsTime) > SMS_INTERVAL) {
      sendSms(buff);
      prevSmsTime = millis();
    }
    
    RTTY.constructString(buff);
    LOG_DEBUG(buff);
    RTTY.write(buff);
    
    for (int i = 0; buff[i] != '\0'; i++) {
      buff[i] = 0;
    }
}

void initRadio() {
  LOG_DEBUG(F("-initRadio(): Initialising radio transmitter."));
  RTTY.attach(RADIO_TX);
  LOG_DEBUG(F("-initRadio(): Done."));  
}

void initGps() {
  LOG_DEBUG(F("-initGps(): Initialising gps."));
  if (gps.init()) {
    LOG_DEBUG(F("-initGps(): Success."));
  } else {
    LOG_DEBUG(F("-initGps(): FAIL: HALTING."));
    exit();
  }
  LOG_DEBUG(F("-initGps(): Done."));
}

void initTempSensors() {
  LOG_DEBUG(F("-initTempSensors(): Initialising sensors."));
  sensors.begin();

  if (!sensors.getAddress(insideThermometer, 0)) LOG_DEBUG(F("-initTempSensors(): Unable to find address for inside thermometer"));
  if (!sensors.getAddress(outsideThermometer, 1)) LOG_DEBUG(F("-initTempSensors(): Unable to find address for outside thermometer"));
  sensors.setResolution(insideThermometer, 11);
  sensors.setResolution(outsideThermometer, 11);
  LOG_DEBUG(F("-initTempSensors(): Sensors initialised."));
}

void initGsm() {
  LOG_DEBUG(F("-initGsm(): Initialising gsm module."));
  sendSms("Initialising... test message.");
  LOG_DEBUG(F("-initGsm(): Done."));
}

void powerGsmOn() {
  LOG_DEBUG(F("-powerGsmOn(): Powering GSM module on."));
  digitalWrite(GSM_POWER, LOW);
  delay(100);
  pinMode(GSM_POWER, OUTPUT);
  digitalWrite(GSM_POWER,HIGH);
  delay(500);
  digitalWrite(GSM_POWER,LOW);
  
  sendCommand("AT\r");
  
  LOG_DEBUG(F("-powerGsmOn(): Done."));
}

void powerGsmOff() {
  LOG_DEBUG(F("-powerGsmOff(): Shutting module down."));
  delay(50);
  gsm.begin(GSM_BAUD);
  gsm.flush();
  delay(50);
  sendCommand("AT+CPOWD=0\r");
  delay(50);
  gsm.end();
  LOG_DEBUG(F("-powerGsmOff(): Done."));
}

void sendCommand(char *s) {
  long start = millis();
  while (true) {
    if (millis() - start > 10000) {
      LOG_DEBUG(F("-sendCommand(): Module not responding."));
      return;
    }
    gsm.write(s);
    delay(100);
    if (gsm.available()) {
      while (gsm.available()) {
        gsm.read();
      }
      return;
    }
    delay(100);
  }
}

void sendSms(char *s) {
  powerGsmOn();
  
  LOG_DEBUG(F("-sendSms(): Sending sms."));
  gsm.begin(GSM_BAUD);
  gsm.print(SMS_SEND_CMD);
  delay(100);
  gsm.print(s);
  delay(100);
  gsm.print((char) 26);
  delay(100);
  gsm.end();
  LOG_DEBUG(F("-sendSms(): Done."));
  
  delay(1000);
  powerGsmOff();
}

void initCompass() {
  LOG_DEBUG(F("-initCompass(): Initialising compass."));
  compass.setScale(1.3);
  compass.setMeasurementMode(Measurement_Continuous);
  LOG_DEBUG(F("-initCompass(): Done."));
}

float getBatteryLevel() {
  float read = (float) analogRead(VOLTAGE_MEASURE_PIN);
  return read / 1023 * 9;
}

unsigned short getHeading() {
  MagnetometerScaled scaled = compass.readScaledAxis();
  
  float heading = atan2(scaled.YAxis, scaled.XAxis);
  
  if(heading < 0)
    heading += 2*PI;

  return (unsigned short) (heading * 180/M_PI); 
}

char* infoToString() {
    char buff[256];
    
    char internalTemp[20];
    char externalTemp[20];
    char batteryLevel[6];
    
    floatToString(payloadInfo.internalTemp, 2, internalTemp);
    floatToString(payloadInfo.externalTemp, 2, externalTemp);
    floatToString(payloadInfo.batteryLevel, 2, batteryLevel);

    snprintf(buff, sizeof(buff), "%s,%u,%s,%s,%s,%ld,%s,%s,%u,%s",
             CALLSIGN,
             payloadInfo.sentenceNumber,
             payloadInfo.gps.time,
             payloadInfo.gps.lat,
             payloadInfo.gps.lon,
             payloadInfo.gps.alt,
             internalTemp,
             externalTemp,
             payloadInfo.heading,
             batteryLevel
            );

    return buff;
}

void floatToString(float f, unsigned char dec, char* buff) {
    dtostrf(f, -1, dec, buff);
}

int freeRam() {
    extern int __heap_start, *__brkval;
    int v;
    return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}

void initSd() {
  LOG_DEBUG(F("-initSd(): Initialising SD."));
  pinMode(SD_CS, OUTPUT);
  SD.begin(SD_CS);
  LOG_DEBUG(F("-initSd(): Done."));
}

#ifdef IS_DEBUG
void logDebug(const __FlashStringHelper *s) {
  Serial.print(millis());
  Serial.print(" ");
  Serial.print(freeRam());
  Serial.print(" ");
  Serial.println(s);
}

void logDebug(const char *s) {
  Serial.print(millis());
  Serial.print(" ");
  Serial.print(freeRam());
  Serial.print(" ");
  Serial.println(s);
}
#endif

File openLogFile() {
  return SD.open(FILENAME, FILE_WRITE);
}

void writeToFile(const char *s, File *file) {
  file->print(millis());
  file->print(" ");
  file->println(s);
}

void exit() {
  while (1);
}

void blinkLed() {
  digitalWrite(STATUS_LED, HIGH);
  delay(200);
  digitalWrite(STATUS_LED, LOW);
  delay(200);
}

