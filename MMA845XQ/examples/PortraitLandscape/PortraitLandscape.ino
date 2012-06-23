/*
 * MMA845XQ test code
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

#include "Wire.h"
#include "MMA845XQ.h"

// If SA0 on MMA845XQ is connected to GND (as on Akafuino L)
MMA845XQ accel;
// If SA0 on MMA845XQ is connected to VCC or floating
//MMA845XQ accel(0x1D);

void setup() {
  Serial.begin(9600);
  Wire.begin();
  accel.begin(false, 2);
}

uint8_t pl_status = 0;

void loop() {
  accel.update();
  pl_status = accel.getPLStatus();
  if(pl_status & 0x80) // Change has occured
  {
    Serial.print("Portrait mode is: ");
    if(pl_status & 0x01) // Bit0 is Back/Front
      Serial.print("Back ");
    else
      Serial.print("Front ");

    if((pl_status & 0x06) == 0x06) // Bit1-2 is Portrait/Landscape mode
      Serial.println("Landscape Left");
    else if((pl_status & 0x04) == 0x04)
      Serial.println("Landscape Right");
    else if((pl_status & 0x02) == 0x02)
      Serial.println("Portrait Down");
    else
      Serial.println("Portrait Up");
  }
  delay(500);
}

