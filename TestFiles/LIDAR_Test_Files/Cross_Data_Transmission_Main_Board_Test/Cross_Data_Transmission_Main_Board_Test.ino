#include <SD.h>
#include <stdint.h>
#include <Wire.h>
#include "LIDARLite_v4LED.h"

const int LightPin =  13;
int x = 0;

void setup() 
{
  Wire.begin();
}

void loop() 
{
  digitalWrite(LightPin, HIGH);
  delay(500);
  digitalWrite(LightPin, LOW);
  delay(500);
  Wire.beginTransmission(45);
  String str = "Testing Run";
  str.concat(x);
  Wire.write(str.c_str());
  String str2 = "Testing_Run";
  str2.concat(x);
  Wire.write(str2.c_str());
  x++;
  Wire.endTransmission();
}
