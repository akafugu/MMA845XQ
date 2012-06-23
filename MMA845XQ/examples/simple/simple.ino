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

uint8_t rdata;
uint8_t i;
int x, x1, y, y1, z, z1;
uint8_t stat;
String stringOutput;

void loop() {
  accel.update();
  Serial.println("X,\tY,\tZ,\tRho,\tPhi,\tTheta");
  Serial.print(accel.getX());
  Serial.print(" , ");
  Serial.print(accel.getY());
  Serial.print(", ");
  Serial.print(accel.getZ());
  Serial.print(", ");
  Serial.print(accel.getRho());
  Serial.print(", ");
  Serial.print(accel.getPhi());
  Serial.print(", ");
  Serial.print(accel.getTheta());
  Serial.println();
  delay(500);
}
