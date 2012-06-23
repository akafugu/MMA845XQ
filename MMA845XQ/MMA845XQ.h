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

#ifndef MMA845XQ_H
#define MMA845XQ_H

#define MMA_845XQ_DEFAULT_ADDRESS 0x1C

#include "Arduino.h"
#include "../Wire/Wire.h"

////////////////////////////////////////////
// Interrupts

// Auto SLEEP/WAKE interrupt
#define INT_ASLP   (1<<7)
// Transient interrupt
#define INT_TRANS  (1<<5)
// Orientation (landscape/portrait) interrupt
#define INT_LNDPRT (1<<4)
// Pulse detection interrupt
#define INT_PULSE  (1<<3)
// Freefall/Motion interrupt
#define INT_FF_MT  (1<<2)
// Data ready interrupt
#define INT_DRDY   (1<<0)


class MMA845XQ
{
  public:
    MMA845XQ(uint8_t addr = MMA_845XQ_DEFAULT_ADDRESS);
    void begin(bool highres = true, uint8_t scale = 2);
    float getX();
    float getY();
    float getZ();
    float getRho();
    float getPhi();
    float getTheta();
    void update();
    
    uint8_t getPLStatus();
    
    uint8_t getPulse();
    
    // Interrupts
    bool setInterrupt(uint8_t type, uint8_t pin, bool on);
    bool disableAllInterrupts();
  private:
	uint8_t _read_register(uint8_t offset);
  	void _write_register(uint8_t b, uint8_t offset);

  	  
    float geta2d(float gx, float gy);
    float geta3d(float gx, float gy, float gz);
    float _getRho(float ax, float ay, float az);
    float _getPhi(float ax, float ay, float az);
    float _getTheta(float ax, float ay, float az);
    
    void _standby();
    void _active();
    
    uint8_t _addr;
    uint8_t _stat;
    uint8_t _scale;
    float _step_factor;
    bool _highres;
    float _xg;
    float _yg;
    float _zg;
    float _rad2deg;
};

#endif

