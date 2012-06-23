/*
 * MMA845XQ library
 * (C) 2012 Akafugu Corporation
 *
 * This program is free software; you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation; either version 2 of the License, or (at your option) any later
 * version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
 * PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 *
 */

#include "MMA845XQ.h"

MMA845XQ::MMA845XQ(uint8_t addr)
{
  _addr = addr;
  _rad2deg = 180.0 / M_PI;
}

//begin private methods

#define MMA_845XQ_CTRL_REG1 0x2A
#define MMA_845XQ_CTRL_REG1_VALUE_ACTIVE 0x01
#define MMA_845XQ_CTRL_REG1_VALUE_F_READ 0x02

#define MMA_845XQ_CTRL_REG2 0x2B
#define MMA_845XQ_CTRL_REG2_RESET 0x40

#define MMA_845XQ_PL_STATUS 0x10
#define MMA_845XQ_PL_CFG 0x11
#define MMA_845XQ_PL_EN 0x40

#define MMA_845XQ_XYZ_DATA_CFG 0x0E
#define MMA_845XQ_2G_MODE 0x00 //Set Sensitivity to 2g
#define MMA_845XQ_4G_MODE 0x01 //Set Sensitivity to 4g
#define MMA_845XQ_8G_MODE 0x02 //Set Sensitivity to 8g

#define MMA_845XQ_FF_MT_CFG 0x15
#define MMA_845XQ_FF_MT_CFG_ELE 0x80
#define MMA_845XQ_FF_MT_CFG_OAE 0x40

#define MMA_845XQ_FF_MT_SRC 0x16
#define MMA_845XQ_FF_MT_SRC_EA 0x80

#define MMA_845XQ_PULSE_CFG 0x21
#define MMA_845XQ_PULSE_CFG_ELE 0x80

#define MMA_845XQ_PULSE_SRC 0x22
#define MMA_845XQ_PULSE_SRC_EA 0x80

uint8_t MMA845XQ::_read_register(uint8_t offset)
{
	Wire.beginTransmission(_addr);
	Wire.write(offset);
	Wire.endTransmission(false);

	Wire.requestFrom(_addr, (uint8_t)1);
	
	if (Wire.available()) return Wire.read();
	return 0;
}

void MMA845XQ::_write_register(uint8_t b, uint8_t offset)
{
	Wire.beginTransmission(_addr);
	Wire.write(offset);
	Wire.write(b);
	Wire.endTransmission();
}


void MMA845XQ::_standby()
{
  uint8_t reg1 = 0x00;
  Wire.beginTransmission(_addr); // Set to status reg
  Wire.write((uint8_t)MMA_845XQ_CTRL_REG1);
  Wire.endTransmission();
  
  Wire.requestFrom((uint8_t)_addr, (uint8_t)1);
  if (Wire.available())
  {
    reg1 = Wire.read();
  }
  Wire.beginTransmission(_addr); // Reset
  Wire.write((uint8_t)MMA_845XQ_CTRL_REG1);
  Wire.write(reg1 & ~MMA_845XQ_CTRL_REG1_VALUE_ACTIVE);
  Wire.endTransmission();
}

void MMA845XQ::_active()
{
  uint8_t reg1 = 0x00;
  Wire.beginTransmission(_addr); // Set to status reg
  Wire.write((uint8_t)MMA_845XQ_CTRL_REG1);
  Wire.endTransmission();
  
  Wire.requestFrom((uint8_t)_addr, (uint8_t)1);
  if (Wire.available())
  {
    reg1 = Wire.read();
  }
  Wire.beginTransmission(_addr); // Reset
  Wire.write((uint8_t)MMA_845XQ_CTRL_REG1);
  Wire.write(reg1 | MMA_845XQ_CTRL_REG1_VALUE_ACTIVE | (_highres ? 0 : MMA_845XQ_CTRL_REG1_VALUE_F_READ) | 0x38);
  Wire.endTransmission();
}

float MMA845XQ::geta2d(float gx, float gy)
{
  float a;
  
  a = gx * gx;
  a = fma(gy,gy,a);
  
  return sqrt(a);
}

//gets the magnitude of the 3d vector
//the formula is a^2 = x^2 + y^2 + z^2
float MMA845XQ::geta3d(float gx, float gy, float gz)
{
  float a;
  
  //use floating point multiply-add cpu func
  //sometimes we get better precision
  a = gx * gx;
  a = fma(gy,gy,a);
  a = fma(gz,gz,a);
  
  return sqrt(a);
}

float MMA845XQ::_getRho(float ax, float ay, float az)
{
  return geta3d(_xg,_yg,_zg);
}

float MMA845XQ::_getPhi(float ax, float ay, float az)
{
  return atan2(ay, ax) * _rad2deg;  
}

float MMA845XQ::_getTheta(float ax, float ay, float az)
{
  float rho = _getRho(ax, ay, az);
  
  if (rho == 0.0)
    return NAN;
  else
    return acos(az / rho) * _rad2deg;
}

//end private methods

#define MMA_845XQ_XYZ_DATA_CFG 0x0E
#define MMA_845XQ_2G_MODE 0x00 //Set Sensitivity to 2g
#define MMA_845XQ_4G_MODE 0x01 //Set Sensitivity to 4g
#define MMA_845XQ_8G_MODE 0x02 //Set Sensitivity to 8g

//begin public methods
void MMA845XQ::begin(bool highres, uint8_t scale)
{
  _highres = highres;
  
  _scale = scale;
  _step_factor = (_highres ? 0.0039 : 0.0156); // Base value at 2g setting
  if( _scale == 4 )
    _step_factor *= 2;
  else if (_scale == 8)
    _step_factor *= 4;
  uint8_t wai = _read_register(0x0D); // Get Who Am I from the device.
  // return value for MMA8543Q is 0x3A
  
  Wire.beginTransmission(_addr); // Reset
  Wire.write(MMA_845XQ_CTRL_REG2);
  Wire.write(MMA_845XQ_CTRL_REG2_RESET);
  Wire.endTransmission();
  delay(10); // Give it time to do the reset
  _standby();
  Wire.beginTransmission(_addr); // Set Portrait/Landscape mode
  Wire.write(MMA_845XQ_PL_CFG);
  Wire.write(0x80 | MMA_845XQ_PL_EN);
  Wire.endTransmission();
  Wire.beginTransmission(_addr);
  Wire.write(MMA_845XQ_XYZ_DATA_CFG);
  if (_scale == 4 || _scale == 8)
    Wire.write((_scale == 4) ? MMA_845XQ_4G_MODE : MMA_845XQ_8G_MODE);
  else // Default to 2g mode
    Wire.write((uint8_t)MMA_845XQ_2G_MODE);
  Wire.endTransmission();
  _active();
}

uint8_t MMA845XQ::getPLStatus()
{
	return _read_register(MMA_845XQ_PL_STATUS);
}

uint8_t MMA845XQ::getPulse()
{
	_write_register(MMA_845XQ_PULSE_CFG, MMA_845XQ_PULSE_CFG_ELE);
	return (_read_register(MMA_845XQ_PULSE_SRC) & MMA_845XQ_PULSE_SRC_EA);
}

float MMA845XQ::getX()
{
  return _xg;
}

float MMA845XQ::getY()
{
  return _yg;
}

float MMA845XQ::getZ()
{
  return _zg;
}

float MMA845XQ::getRho()
{
  return _getRho(_xg,_yg,_zg);
}

float MMA845XQ::getPhi()
{
  return _getPhi(_xg,_yg,_zg);
}

float MMA845XQ::getTheta()
{
  return _getTheta(_xg,_yg,_zg);
}

int16_t rx, ry, rz;

void MMA845XQ::update()
{
  Wire.beginTransmission(_addr); // Set to status reg
  Wire.write((uint8_t)0x00);
  Wire.endTransmission(false);
  
  Wire.requestFrom((uint8_t)_addr, (uint8_t)(_highres ? 7 : 4));
  if (Wire.available()) 
  {
    _stat = Wire.read();
    if(_highres)
    {
      rx = (int16_t)((Wire.read() << 8) + Wire.read());
      _xg = (rx / 64) * _step_factor;
      ry = (int16_t)((Wire.read() << 8) + Wire.read());
      _yg = (ry / 64) * _step_factor;
      rz = (int16_t)((Wire.read() << 8) + Wire.read());
      _zg = (rz / 64) * _step_factor;
    }
    else
    {
      _xg = (int8_t)Wire.read()*_step_factor;
      _yg = (int8_t)Wire.read()*_step_factor;
      _zg = (int8_t)Wire.read()*_step_factor;
    }
  }
}

bool MMA845XQ::setInterrupt(uint8_t type, uint8_t pin, bool on)
{
	uint8_t current_value = _read_register(0x2D);
	
	if(on)
		current_value |= type;
	else
		current_value &= ~(type);
	
	_write_register(0x2D, current_value);
	
	uint8_t current_routing_value = _read_register(0x2E);
	
	if (pin == 1) {
		current_routing_value &= ~(type);
	}
	else if (pin == 2) {
		current_routing_value |= type;
	}
	
	_write_register(0x2E, current_routing_value);
}

bool MMA845XQ::disableAllInterrupts()
{
	_write_register(0x2D, 0);
}



//end public methods
