//#define DEBUG 1 //Uncomment to enable debug messages to the debug pin
#ifdef DEBUG
#define LOG_DEBUG(x) debug_serial.print(x)
#else
#define LOG_DEBUG(x) void(0)
#endif

#include <SoftwareSerial.h>
#include <util/crc16.h>

#define DEBUG_RX 5
#define DEBUG_TX 8

#define GPS_BAUD 4800

#define RADIO_TX 3

const char CALLSIGN[] = "$$MEURS";

const uint8_t SENTENCE_LENGTH = 128;
char sentence_buff[SENTENCE_LENGTH];

uint16_t sentenceNumber = 0;
char time[9];
char lat[13];
char lon[14];
uint32_t alt;

#ifdef DEBUG
SoftwareSerial debug_serial(DEBUG_RX, DEBUG_TX);
#endif

void setup() {
	digitalWrite(9, HIGH);
	delay(100);
	digitalWrite(9, LOW);
#ifdef DEBUG
	debug_serial.begin(4800);
	delay(100);
#endif
	LOG_DEBUG(F("Initialising.\n"));
    radio_init();
    gps_init();
	LOG_DEBUG(F("Done.\n"));
	digitalWrite(9, HIGH);
}

void loop() {
    sentenceNumber++;
	memset(sentence_buff, '\0', SENTENCE_LENGTH);
	
	LOG_DEBUG(F("Polling GPS...\n"));
    if (gps_poll()){
		infoToString();
		LOG_DEBUG('\n');
		LOG_DEBUG(sentence_buff);
		radio_write(sentence_buff);
	}
}

void radio_init() {
	LOG_DEBUG(F("Initialising radio... "));
    pinMode(RADIO_TX, OUTPUT);
    setPwmFrequency(RADIO_TX, 1);
	LOG_DEBUG(F("Done.\n"));
}

// Send a byte array to the GPS
void gps_writeArray(uint8_t *MSG, uint8_t len) {
    for(int i=0; i<len; i++) {
        Serial.write(MSG[i]);
    }
    Serial.println();
}


// Calculate expected UBX ACK packet and parse UBX response from GPS
// Taken from http://ukhas.org.uk/guides:ublox6 and modified
boolean gps_getUbxAck(uint8_t *MSG) {
    uint8_t b;
    uint8_t ackByteID = 0;
    uint8_t ackPacket[10];
    unsigned long startTime = millis();
	
    // Construct the expected ACK packet
    ackPacket[0] = 0xB5;	// header
    ackPacket[1] = 0x62;	// header
    ackPacket[2] = 0x05;	// class
    ackPacket[3] = 0x01;	// id
    ackPacket[4] = 0x02;	// length
    ackPacket[5] = 0x00;
    ackPacket[6] = MSG[2];	// ACK class
    ackPacket[7] = MSG[3];	// ACK id
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
            // All packets in order!
            return true;
        }

        // Timeout if no valid response in 3 seconds
        if (millis() - startTime > 3000) {
            return false;
        }

        // Make sure data is available to read
        if (Serial.available()) {
            b = Serial.read();
            // Check that bytes arrive in sequence as per expected ACK packet
            if (b == ackPacket[ackByteID]) {
                ackByteID++;
            } else {
                ackByteID = 0;	// Reset and look again, invalid order
            }

        }
    }
}

bool gps_init() {
	LOG_DEBUG(F("Initialising gps...\n"));

	Serial.begin(9600);
	Serial.print(F("$PUBX,41,1,0007,0003,4800,0*13\r\n"));
	Serial.flush();
	Serial.end();

	Serial.begin(GPS_BAUD);

	LOG_DEBUG("Setting uBlox nav mode...\n");
	uint8_t setNav[] = {
		0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00,
		0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC
	};

	do {
		gps_writeArray(setNav, sizeof(setNav));
	} while (!gps_getUbxAck(setNav));
	
	LOG_DEBUG("Switching off NMEA GGA...\n");
	uint8_t setGGA[] = {
		0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x23
	};
	
	do {
		gps_writeArray(setGGA, sizeof(setGGA));
	} while (!gps_getUbxAck(setGGA));

	LOG_DEBUG("Switching off NMEA GLL...\n");
	uint8_t setGLL[] = {
		0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x2B
	};

	do {
		gps_writeArray(setGLL, sizeof(setGLL));
	} while (!gps_getUbxAck(setGLL));

	LOG_DEBUG("Switching off NMEA GSA...\n");
	uint8_t setGSA[] = {
		0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x32
	};

	do {
		gps_writeArray(setGSA, sizeof(setGSA));
	} while (!gps_getUbxAck(setGSA));

	LOG_DEBUG("Switching off NMEA GSV...\n");
	uint8_t setGSV[] = {
		0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x39
	};

	do {
		gps_writeArray(setGSV, sizeof(setGSV));
	} while (!gps_getUbxAck(setGSV));

	LOG_DEBUG("Switching off NMEA RMC...\n");
	uint8_t setRMC[] = {
		0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x04, 0x40
	};

	do {
		gps_writeArray(setRMC, sizeof(setRMC));
	} while (!gps_getUbxAck(setRMC));
   
	LOG_DEBUG("Switching off NMEA VTG...\n");
	uint8_t setVTG[] = {
		0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x46
	};
	
	do {
		gps_writeArray(setVTG, sizeof(setVTG));
	} while (!gps_getUbxAck(setVTG));
		
    return true;
}

// Empties the hardware serial transmit and receive buffers.
void gps_emptyBuffer() {
	Serial.flush();
	while (Serial.available()) {
		Serial.read();		
	}	
}

// Extracts the used info from the GPS module's response and converts this info to the necessary format.
bool gps_tokenise(char *buff) {
	char *pt;
	uint8_t index = 0;
	for (pt = strtok(buff, ","); pt; pt = strtok(NULL, ",")) {
		switch (index) {
		case 0:
			if (strcmp(pt, "$PUBX") != 0) {
				return false;
			}
			break;
		case 2:
			strcpy(time, "00:00:00");
			memcpy(time, pt, 2);
			memcpy(time + 3, pt + 2, 2);
			memcpy(time + 6, pt + 4, 2);
			LOG_DEBUG(F("\nTime: "));
			LOG_DEBUG(time);
			break;
			
		case 3:
			strcpy(lat, pt);
			break;
			
		case 4:
			if (*pt == 'S') {
				memcpy(lat + 1, lat, 12);
				lat[0] = '-';
			}
			LOG_DEBUG(F("\nLatitude: "));
			LOG_DEBUG(lat);
			break;
			
		case 5:
			strcpy(lon, pt);
			break;
			
		case 6:
			if (*pt == 'W') {
				memcpy(lon + 1, lon, 13);
				lon[0] = '-';
			}
			LOG_DEBUG(F("\nLongitude: "));
			LOG_DEBUG(lon);
			break;
			
		case 7:;
			uint8_t i = 0;
			for (i = 0; i < strlen(pt); i++) {
				if (pt[i] == '.')
				pt[i] = '\0';
			}
			alt = strtol(pt, NULL, 10);
			LOG_DEBUG(F("\nAltitude: "));
			LOG_DEBUG(alt);
			return true;
			
		}
		index++;
	}
	return false;
}

// Polls the GPS module to update flight data
bool gps_poll() {
	 unsigned long startTime = millis();
	 unsigned short timeout = 20000;

	 char buff[128];

	 char b = 0;
	 uint8_t len = 0;

	 uint8_t parity = 0;

	 uint8_t checksum = 0;
	 bool isChecksum = false;
	 bool hasChecksumFirstChar = false;
	 bool hasChecksumSecondChar = false;
	
	gps_emptyBuffer();
	
	// Tickle the GPS
	Serial.println(F("$PUBX,00*33"));
	while (!Serial.available());
	
    while (Serial.available() && b != '\n') {
        if (millis() - startTime > timeout) {
			LOG_DEBUG(F("timeout!\n"));
            return false;
        }
		
        b = Serial.read();
		
        LOG_DEBUG(b);
		
		// The method used here to check the checksum is rather ugly...
        if (b == '*') {
	        isChecksum = true;
	        } else if (isChecksum && !hasChecksumFirstChar) {
	        checksum += 16 * gps_fromHex(b);
	        hasChecksumFirstChar = true;
	        } else if (isChecksum && !hasChecksumSecondChar) {
	        checksum += gps_fromHex(b);
	        hasChecksumSecondChar = true;
        }

        if (!isChecksum && b != '$' && b != '\r' && b != '\n') parity ^= b;

        buff[len++] = b;
    }
	
    buff[len] = 0x00;

    if (parity != checksum) {
		LOG_DEBUG(F("checksum failed!\n"));
        return false;
    }
	
	return gps_tokenise(buff);
}
// Converts a hexadecimal character to an unsigned byte
uint8_t gps_fromHex(char a) {
    if (a >= 'A' && a <= 'F')
        return a - 'A' + 10;
    else if (a >= 'a' && a <= 'f')
        return a - 'a' + 10;
    else
        return a - '0';
}

// Constructs an RTTY sentence by combining all data into one string
char* infoToString() {
    snprintf(sentence_buff, SENTENCE_LENGTH, "%s,%u,%s,%s,%s,%u",
             CALLSIGN,
             sentenceNumber,
             time,
             lat,
             lon,
             alt
    );
			
	char checksum[7];
	snprintf(checksum, 7, "*%04X\n", generateChecksum(sentence_buff));

	memcpy(sentence_buff + strlen(sentence_buff), checksum, strlen(checksum) + 1);
}

// Taken from http://playground.arduino.cc/Code/PwmFrequency
// Improves the quality of the NTX2B radio signal a LOT
void setPwmFrequency(int pin, int divisor) {
    byte mode;
    if(pin == 5 || pin == 6 || pin == 9 || pin == 10) {
        switch(divisor) {
        case 1:
            mode = 0x01;
            break;
        case 8:
            mode = 0x02;
            break;
        case 64:
            mode = 0x03;
            break;
        case 256:
            mode = 0x04;
            break;
        case 1024:
            mode = 0x05;
            break;
        default:
            return;
        }
        if(pin == 5 || pin == 6) {
            TCCR0B = TCCR0B & 0b11111000 | mode;
        } else {
            TCCR1B = TCCR1B & 0b11111000 | mode;
        }
    } else if(pin == 3 || pin == 11) {
        switch(divisor) {
        case 1:
            mode = 0x01;
            break;
        case 8:
            mode = 0x02;
            break;
        case 32:
            mode = 0x03;
            break;
        case 64:
            mode = 0x04;
            break;
        case 128:
            mode = 0x05;
            break;
        case 256:
            mode = 0x06;
            break;
        case 1024:
            mode = 0x7;
            break;
        default:
            return;
        }
        TCCR2B = TCCR2B & 0b11111000 | mode;
    }
}

// Writes one string to the NTX2B
void radio_write(char * string) {
    char c;

    c = *string++;

    while ( c != '\0') {
        rtty_txbyte (c);
        c = *string++;
    }
}

// Writes one byte to the NTX2B
void rtty_txbyte (char c) {
    int i;

    rtty_txbit(0); // Start bit
	
    for (i = 0; i < 7; i++) {
        if (c & 1) rtty_txbit(1);
        else rtty_txbit(0);

        c = c >> 1;
    }

    rtty_txbit(1); // Stop bit
}

//Writes one bit to the NTX2B
void rtty_txbit (int bit) {
    if (bit) {
        // high
        analogWrite(RADIO_TX, 110);

    } else {
        // low
        analogWrite(RADIO_TX, 100);
    }
	
	// 20150 microseconds = 50 baud
    delayMicroseconds(10000);
    delayMicroseconds(10150);
}

// Generates a checksum for the given data sentence
unsigned short generateChecksum(const char *s) {
    unsigned short crc = 0xFFFF;
    for (unsigned char i = 0; s[i] != 0x00; i++) {
        if (s[i] == '$') continue;
        crc = _crc_xmodem_update(crc, s[i]);
    }
    return crc;
}