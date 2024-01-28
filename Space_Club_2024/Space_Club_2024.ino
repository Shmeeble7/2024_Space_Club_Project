/*
 * 
 * Insert basic team and project info
 * dont forget this time
 * 
 */
 
#include <stdint.h>
#include <Wire.h>
#include <WireIMXRT.h>
#include <WireKinetis.h>
#include "LIDARLite_v4LED.h"
#include <Adafruit_Sensor.h>
#include <SD.h>
#include <L3G.h>
//Gyroscope, 3 axis, records rotation data
#include <Adafruit_L3GD20.h>
#include <Adafruit_L3GD20_U.h>
//Magnetometer, records and measures magnetic fields
#include <Adafruit_MMC56x3.h>
//Accelerometer, 3 axis, records acceleration on all three axis.
#include <Adafruit_ADXL375.h>

//Our gyroscope object
Adafruit_L3GD20 gyro;

Adafruit_MMC56x3 magnet;

Adafruit_ADXL375 accelerometer;

int sd = BUILTIN_SDCARD;




//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void setup() 
{
  int z = 0;

}

void loop() 
{
  // put your main code here, to run repeatedly:

}
