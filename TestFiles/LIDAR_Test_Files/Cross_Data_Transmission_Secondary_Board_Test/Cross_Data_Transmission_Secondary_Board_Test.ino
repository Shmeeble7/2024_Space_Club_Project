
#include <SD.h>
#include <stdint.h>
#include <Wire.h>
#include "LIDARLite_v4LED.h"

File CrossDataTest;

//Our sd card
int sd = BUILTIN_SDCARD;

const int LightPin =  13;

volatile bool dataRecieved = false;

volatile char x[32];

void setup() 
{
  Wire.begin(45);
  if(!SD.begin(sd))
  {
    while(1)
    {
      digitalWrite(LightPin, HIGH);
      delay(1000);
      digitalWrite(LightPin, LOW);
      delay(1000);
    }
  }
  Wire.onReceive(temp);
}

void loop() 
{
  digitalWrite(LightPin, HIGH);
  delay(100);
  digitalWrite(LightPin, LOW);
  delay(100);
  if(dataRecieved)
  {
    CrossDataTest = SD.open("CrossDataTest.txt", FILE_WRITE);

    if (CrossDataTest)
    {

      for(int i = 0; i < 32; i++)
      {
        CrossDataTest.print(x[i]);
      }
        CrossDataTest.print("T: ");
        CrossDataTest.println(millis() / 1000.0);
        CrossDataTest.close();
    }
    dataRecieved = false;
  }
  else
  {
    CrossDataTest = SD.open("CrossDataTest.txt", FILE_WRITE);

    if (CrossDataTest)
    {
      
      //CrossDataTest.println("No data recieved");
      CrossDataTest.close();
    }
  }
}

void temp(int howMany)
{
  for(int i = 0; i < howMany; i++)
  {
    x[i]= Wire.read();
  }
  dataRecieved = true;
  
}
