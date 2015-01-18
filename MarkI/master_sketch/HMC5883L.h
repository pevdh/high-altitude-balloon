/*
HMC5883L.h - Header file for the HMC5883L Triple Axis Magnetometer Arduino Library.
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

#ifndef HMC5883L_h
#define HMC5883L_h

#include <inttypes.h>
#include <Wire.h>

#define HMC5883L_Address 0x1E
#define ConfigurationRegisterA 0x00
#define ConfigurationRegisterB 0x01
#define ModeRegister 0x02
#define DataRegisterBegin 0x03
#define IdentityRegister 0x0A
#define IdentityRegisterValue 0x48

#define Measurement_Continuous 0x00
#define Measurement_SingleShot 0x01
#define Measurement_Idle 0x03

struct MagnetometerScaled
{
	float XAxis;
	float YAxis;
	float ZAxis;
};

struct MagnetometerRaw
{
	int XAxis;
	int YAxis;
	int ZAxis;
};

class HMC5883L
{
	public:
	  HMC5883L();

	  MagnetometerRaw readRawAxis();
	  MagnetometerScaled readScaledAxis();
  
	  void setMeasurementMode(uint8_t mode);
	  void setScale(float gauss);

	  uint8_t ensureConnected();
	  uint8_t isConnected;
	protected:

	private:
	  float m_Scale;

	  void write(int address, int byte);
	  uint8_t* read(int address, int length);
};
#endif
