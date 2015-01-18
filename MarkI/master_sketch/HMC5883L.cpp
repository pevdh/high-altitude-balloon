/*
HMC5883L.cpp - Class file for the HMC5883L Triple Axis Magnetometer Arduino Library.
Copyright (C) 2011 Love Electronics (bluelemonlabs.blogspot.com)

This program is free software: you can redistribute it and/or modify
it under the terms of the version 3 GNU General Public License as
published by the Free Software Foundation.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

 WARNING: THE HMC5883L IS NOT IDENTICAL TO THE HMC5883!
 Datasheet for HMC5883L:
 http://www51.honeywell.com/aero/common/documents/myaerospacecatalog-documents/Defense_Brochures-documents/HMC5883L_3-Axis_Digital_Compass_IC.pdf
 http://c48754.r54.cf3.rackcdn.com/HMC5883L.pdf

*/

#include <Arduino.h>
#include "HMC5883L.h"
#define EPSILON 0.001


HMC5883L::HMC5883L()
{
  m_Scale = 1.3;
  Wire.begin();
}

MagnetometerRaw HMC5883L::readRawAxis()
{
  uint8_t* buffer = read(DataRegisterBegin, 6);
  MagnetometerRaw raw = MagnetometerRaw();
  raw.XAxis = (buffer[0] << 8) | buffer[1];
  raw.ZAxis = (buffer[2] << 8) | buffer[3];
  raw.YAxis = (buffer[4] << 8) | buffer[5];
  return raw;
}

MagnetometerScaled HMC5883L::readScaledAxis()
{
  MagnetometerRaw raw = readRawAxis();
  MagnetometerScaled scaled = MagnetometerScaled();
  scaled.XAxis = raw.XAxis * m_Scale;
  scaled.ZAxis = raw.ZAxis * m_Scale;
  scaled.YAxis = raw.YAxis * m_Scale;
  return scaled;
}

void HMC5883L::setScale(float gauss)
{
	uint8_t regValue = 0x00;
	if( abs(gauss - 0.88 < EPSILON))
	{
		regValue = 0x00;
		m_Scale = 0.73;
	}
	else if(abs(gauss - 1.3 < EPSILON))
	{
		regValue = 0x01;
		m_Scale = 0.92;
	}
	else if(abs(gauss - 1.9 < EPSILON))
	{
		regValue = 0x02;
		m_Scale = 1.22;
	}
	else if(abs(gauss - 2.5 < EPSILON))
	{
		regValue = 0x03;
		m_Scale = 1.52;
	}
	else if(abs(gauss - 4.0 < EPSILON))
	{
		regValue = 0x04;
		m_Scale = 2.27;
	}
	else if(abs(gauss - 4.7 < EPSILON))
	{
		regValue = 0x05;
		m_Scale = 2.56;
	}
	else if(abs(gauss - 5.6 < EPSILON))
	{
		regValue = 0x06;
		m_Scale = 3.03;
	}
	else if(abs(gauss - 8.1 < EPSILON))
	{
		regValue = 0x07;
		m_Scale = 4.35;
	}
	
	// Setting is in the top 3 bits of the register.
	regValue = regValue << 5;
	write(ConfigurationRegisterB, regValue);
}

void HMC5883L::setMeasurementMode(uint8_t mode)
{
	write(ModeRegister, mode);
}

uint8_t HMC5883L::ensureConnected()
{
	uint8_t data = read(IdentityRegister, 1)[0];

	if(data == IdentityRegisterValue)
		isConnected = 1;
	else
		isConnected = 0;

	return isConnected;
}

void HMC5883L::write(int address, int data)
{
  Wire.beginTransmission(HMC5883L_Address);
  Wire.write(address);
  Wire.write(data);
  Wire.endTransmission();
}

uint8_t* HMC5883L::read(int address, int length)
{
  Wire.beginTransmission(HMC5883L_Address);
  Wire.write(address);
  Wire.endTransmission();
  
  Wire.beginTransmission(HMC5883L_Address);
  Wire.requestFrom(HMC5883L_Address, length);

  uint8_t buffer[length];
  if(Wire.available() == length)
  {
	  for(uint8_t i = 0; i < length; i++)
	  {
		  buffer[i] = Wire.read();
	  }
  }
  Wire.endTransmission();

  return buffer;
}
