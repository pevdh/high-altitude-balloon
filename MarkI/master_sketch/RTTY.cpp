/*
 * RTTY.cpp
 * Version 0.1 - http://www.timzaman.com/
 * Copyright (c) 2011 Tim Zaman
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE FOUNDATION OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Code based in part on information from the following:
 * [J. Coxon, D.Richman]
 */
 
#include "RTTY.h"
#include <Arduino.h>
#include <string.h>
#include <util/crc16.h>

#define ASCII_BIT 7
#define BAUD_RATE 20150     // 10000 = 100 BAUD 20150
//Dont mind these two, they'll be overwritten.
int pa=2;
/* PUBLIC METHODS */

RTTY5::RTTY5()
{
}

/**
 * Divides a given PWM pin frequency by a divisor.
 * 
 * The resulting frequency is equal to the base frequency divided by
 * the given divisor:
 *   - Base frequencies:
 *      o The base frequency for pins 3, 9, 10, and 11 is 31250 Hz.
 *      o The base frequency for pins 5 and 6 is 62500 Hz.
 *   - Divisors:
 *      o The divisors available on pins 5, 6, 9 and 10 are: 1, 8, 64,
 *        256, and 1024.
 *      o The divisors available on pins 3 and 11 are: 1, 8, 32, 64,
 *        128, 256, and 1024.
 * 
 * PWM frequencies are tied together in pairs of pins. If one in a
 * pair is changed, the other is also changed to match:
 *   - Pins 5 and 6 are paired on timer0
 *   - Pins 9 and 10 are paired on timer1
 *   - Pins 3 and 11 are paired on timer2
 * 
 * Note that this function will have side effects on anything else
 * that uses timers:
 *   - Changes on pins 3, 5, 6, or 11 may cause the delay() and
 *     millis() functions to stop working. Other timing-related
 *     functions may also be affected.
 *   - Changes on pins 9 or 10 will cause the Servo library to function
 *     incorrectly.
 * 
 * Thanks to macegr of the Arduino forums for his documentation of the
 * PWM frequency divisors. His post can be viewed at:
 *   http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1235060559/0#4
 */
void setPwmFrequency(int pin, int divisor) {
  byte mode;
  if(pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if(pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    } else {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  } else if(pin == 3 || pin == 11) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 32: mode = 0x03; break;
      case 64: mode = 0x04; break;
      case 128: mode = 0x05; break;
      case 256: mode = 0x06; break;
      case 1024: mode = 0x7; break;
      default: return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
}

void RTTY5::attach(int pina)
{
	pa=pina;
	pinMode(pa, OUTPUT);
        setPwmFrequency(pa, 1);
}


void RTTY5::write(char * string)
{
	/* Simple function to sent a char at a time to 
	** rtty_txbyte function. 
	** NB Each char is one byte (8 Bits)
	*/

	char c;

	c = *string++;

	while ( c != '\0')
	{
		rtty_txbyte (c);
		c = *string++;
	}
        //analogWrite(pa, 100);
}


void RTTY5::rtty_txbyte (char c)
{
	int i;

	rtty_txbit (0); // Start bit

	// Send bits for for char LSB first	

	for (i=0;i<ASCII_BIT;i++)
	{
		if (c & 1) rtty_txbit(1); 

			else rtty_txbit(0);	

		c = c >> 1;

	}

	rtty_txbit (1); // Stop bit
}

void RTTY5::rtty_txbit (int bit)
{
		if (bit)
		{
		  // high
                    analogWrite(pa, 110);
                    
		}
		else
		{
		  // low
                    analogWrite(pa, 100);
		}
   delayMicroseconds(10000);
   delayMicroseconds(10150);
}

void RTTY5::constructString(char* buff) {
    char checksum[7];
    snprintf(checksum, 7, "*%04X\n", generateChecksum(buff));
    //strcat(buff, checksum);
    
    memcpy(buff + strlen(buff), checksum, strlen(checksum) + 1);
}

unsigned short RTTY5::generateChecksum(const char *s) {
    unsigned short crc = 0xFFFF;
    for (unsigned char i = 0; s[i] != 0x00; i++) {
        if (s[i] == '$') continue;
        crc = _crc_xmodem_update(crc, s[i]);
    }
    return crc;
}

// Set the default object
RTTY5 RTTY = RTTY5();
