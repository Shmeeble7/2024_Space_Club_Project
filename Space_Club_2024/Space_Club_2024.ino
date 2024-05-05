/* 
 *  2024 Space Club Primary Board 
 *  Created by Ian Bailey and Ryan Hayden for the WVU Tech 2024 Space Club project
 *  This version of the file should handle running the lidar in continuous mode without additional
 *  input in the serial monitor and it should properly save data into a files on the built-in SD card.
 *  This file should handle all 4 Lidar modules but additional testing may be required
*/

//Functional version as of 4/23/24

//These are the libraries need
#include <SD.h>
#include <stdint.h>
#include <Wire.h>
#include "LIDARLite_v4LED.h"

//The i2c addresses for each board, used for sending data to each other
#define This_Address 30
#define Other_Address 45

//The I2C addresses were hoping to use
char LIDAR_1_ADDR = 0x64;
char LIDAR_2_ADDR = 0x74;
char LIDAR_3_ADDR = 0x75;
char LIDAR_4_ADDR = 0x76;

//These four objects represent all four of our lidars, 
LIDARLite_v4LED Lidar1;
LIDARLite_v4LED Lidar2;
LIDARLite_v4LED Lidar3;
LIDARLite_v4LED Lidar4;

#define FAST_I2C

//define which pins are the monitor and trigger pins
#define MonitorPin    3
#define TriggerPin    2

//Initialize the sd card
int sd = BUILTIN_SDCARD;

//Initialize the data files, these are only used when we run the board by itself, mainly for testing purposes
File Lidar_1_Data;
File Lidar_2_Data;
File Lidar_3_Data;
File Lidar_4_Data;


//define the LED pin
const int LightPin =  13;

//This enum datatype is used to hold all the range types simultaneously
//The GPIO function worka the same but uses the GPIO pin on the LIDAR to function
enum rangeType_T
{
  RANGE_NONE,
  RANGE_CONTINUOUS,
  RANGE_CONTINUOUS_GPIO
};

//==========================
//Setup Function
//==========================
//*************************************************************************************************************************************************************************
void setup()
{
  // Initialize Arduino serial port (for display of ASCII output to PC)
  Serial.begin(115200);

  // Initialize Arduino I2C (for communication to LidarLite)
  Wire.begin();
#ifdef FAST_I2C
#if ARDUINO >= 157
  Wire.setClock(400000UL); // Set I2C frequency to 400kHz
#else
  TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
#endif
#endif

//Set the first lidar I2C address
//*************************************************************************************************************************************************************************
Lidar1.setI2Caddr(LIDAR_1_ADDR, 1, LIDAR_1_ADDR);
Lidar2.setI2Caddr(LIDAR_2_ADDR, 1, LIDAR_2_ADDR);
Lidar3.setI2Caddr(LIDAR_3_ADDR, 1, LIDAR_3_ADDR);
Lidar4.setI2Caddr(LIDAR_4_ADDR, 1, LIDAR_4_ADDR);


// ----------------------------------------------------------------------
// Lights and SD card setup
// ----------------------------------------------------------------------
//*************************************************************************************************************************************************************************
// initialize the digital pin as an output
  pinMode(LightPin, OUTPUT);

  if (!SD.begin(sd))
  {
    //This will blink the built-in LED once every second if the SD card fails to initialize
    while (1)
    {
      digitalWrite(LightPin, HIGH);
      delay(1000);
      digitalWrite(LightPin, LOW);
      delay(1000);
    }
  }
  
//These will check if the LIDAR data files initialize properly, we dont need these for the actual project
//since they are only used in single board testing. Ive left these lines uncommented since they shouldnt cause issues.
  Lidar_1_Data = SD.open("Lidar_1_Data.txt", FILE_WRITE);

  if (Lidar_1_Data)
  {
    Lidar_1_Data.println("Lidar_1_File successfully initialized");
    Lidar_1_Data.close();
  }

  Lidar_2_Data = SD.open("Lidar_2_Data.txt", FILE_WRITE);

  if (!Lidar_2_Data)
  {
    Lidar_2_Data.println("Lidar_2_File successfully initialized");
    Lidar_2_Data.close();
  }

  Lidar_3_Data = SD.open("Lidar_3_Data.txt", FILE_WRITE);

  if (Lidar_3_Data)
  {
    Lidar_3_Data.println("Lidar_3_File successfully initialized");
    Lidar_3_Data.close();
  }

  Lidar_4_Data = SD.open("Lidar_4_Data.txt", FILE_WRITE);

  if (Lidar_4_Data)
  {
    Lidar_4_Data.println("Lidar_4_File successfully initialized");
    Lidar_4_Data.close();
  }

// ----------------------------------------------------------------------
// Additional Setup/Info
// ----------------------------------------------------------------------
//*************************************************************************************************************************************************************************

  // ----------------------------------------------------------------------
  // The LIDAR-Lite v4 LED is strictly a 3.3V system. The Arduino Due is a
  // 3.3V system and is recommended for use with the LIDAR-Lite v4 LED. Teensy is
  // also a 3.3v Board. Care MUST be taken if connecting to a 5V system such as the Arduino Uno.


  // The two digitalWrite() functions (below) turn off the micro's internal
  // pull-up resistors. This protects the LLv4 from damage via overvoltage
  // but requires external pullups to 3.3V for the I2C signals.

  // ----------------------------------------------------------------------
  digitalWrite(SCL, LOW);
  digitalWrite(SDA, LOW);

  // ----------------------------------------------------------------------
  // Optional GPIO pin assignments for measurement triggering & monitoring
  // ----------------------------------------------------------------------
  pinMode(MonitorPin, INPUT);
  pinMode(TriggerPin, OUTPUT);
  digitalWrite(TriggerPin, LOW);

  // ----------------------------------------------------------------------
  // Optionally configure the LidarLite parameters to lend itself to
  // various modes of operation by altering 'configure' input integer.
  //
  // configuration:  Default 0.
  // 0: Maximum range. Uses maximum acquisition count.
  // 1: Balanced performance.
  // 2: Short range, high speed. Reduces maximum acquisition count.
  // 3: Mid range, higher speed. Turns on quick termination
  //    detection for faster measurements at short range (with decreased
  //    accuracy)
  // 4: Maximum range, higher speed on short range targets. Turns on quick
  //    termination detection for faster measurements at short range (with
  //    decreased accuracy)
  // 5: Very short range, higher speed, high error. Reduces maximum
  //    acquisition count to a minimum for faster rep rates on very
  //    close targets with high error.
  // ----------------------------------------------------------------------
  Lidar1.configure(0);
  Lidar2.configure(0);
  Lidar3.configure(0);
  Lidar4.configure(0);

}
//======================
//loop function
//======================
//*************************************************************************************************************************************************************************
void loop()
{
  //despite being in the loop function this block will only be triggered once due to the while loop down below
  uint16_t distance;
  uint8_t  Lidar1_newDistance;
  uint8_t  Lidar2_newDistance;
  uint8_t  Lidar3_newDistance;
  uint8_t  Lidar4_newDistance;
  uint8_t  inputChar;

  rangeType_T rangeMode = RANGE_CONTINUOUS;

  

  //This is where the actual looping occurs
  while (1)
  {

    //===================================================================
    // 2) Call the distance methods for each Lidar
    //===================================================================

    //Im only keeping this as a switch incase we want to stop recording data temporarily for some reason
    switch (rangeMode)
    {
      case RANGE_NONE:
        Lidar1_newDistance = 0;
        Lidar2_newDistance = 0;
        Lidar3_newDistance = 0;
        Lidar4_newDistance = 0;
        break;

      case RANGE_CONTINUOUS:
        Lidar1_newDistance = Lidar1_distanceContinuous(&distance);
        Lidar2_newDistance = Lidar2_distanceContinuous(&distance);
        Lidar3_newDistance = Lidar3_distanceContinuous(&distance);
        Lidar4_newDistance = Lidar4_distanceContinuous(&distance);
        break;

      default:
        //If this activates there is something wrong
        Lidar1_newDistance = -1000;
        Lidar2_newDistance = -1000;
        Lidar3_newDistance = -1000;
        Lidar4_newDistance = -1000;
        rangeMode = RANGE_NONE;
        break;
    }

    //===================================================================
    // 3) When there is new distance data, print it to file on the SD card
    //===================================================================
    if (Lidar1_newDistance)
    {
      
      //=========================================================================================
      // This comment block can be used to get the Lidar data when running the file in solo mode
      //=========================================================================================
      /*Lidar_1_Data = SD.open("Lidar_1_Data.txt", FILE_WRITE);

      //Send the data from the lidar to the text file above
      if (Lidar_1_Data)
      {
        Lidar_1_Data.print("Lidar 1 distance is: "); Lidar_1_Data.print(Lidar1_newDistance); Lidar_1_Data.println("cm");
        
        //Close the SD Card
        Lidar_1_Data.close();
      }*/
      
      String Lidar_1_str = "Lidar 1 distance is: ";
      Lidar_1_str.append(Lidar1_newDistance);

      Wire.beginTransmission(Other_Address);
      Wire.write(Lidar_1_str.c_str());
      Wire.endTransmission();
    }

     if (Lidar2_newDistance)
    {

      //=========================================================================================
      // This comment block can be used to get the Lidar data when running the file in solo mode
      //=========================================================================================
      /*Lidar_2_Data = SD.open("Lidar_2_Data.txt", FILE_WRITE);

      //Send the data from the lidar to the text file above
      if (Lidar_2_Data)
      {
        Lidar_2_Data.print("Lidar 2 distance is: "); Lidar_2_Data.print(Lidar2_newDistance); Lidar_2_Data.println("cm");
        
        //Close the SD Card
        Lidar_2_Data.close();
      }*/
      
      String Lidar_2_str = "Lidar 2 distance is: ";
      Lidar_2_str.append(Lidar2_newDistance);

      Wire.beginTransmission(Other_Address);
      Wire.write(Lidar_2_str.c_str());
      Wire.endTransmission();
    }

     if (Lidar3_newDistance)
    {

      //=========================================================================================
      // This comment block can be used to get the Lidar data when running the file in solo mode
      //=========================================================================================
      /*Lidar_3_Data = SD.open("Lidar_3_Data.txt", FILE_WRITE);

      //Send the data from the lidar to the text file above
      if (Lidar_3_Data)
      {
        Lidar_3_Data.print("Lidar 3 distance is: "); Lidar_3_Data.print(Lidar3_newDistance); Lidar_3_Data.println("cm");
        
        //Close the SD Card
        Lidar_3_Data.close();
      }*/

      String Lidar_3_str = "Lidar 3 distance is: ";
      Lidar_3_str.append(Lidar3_newDistance);

      Wire.beginTransmission(Other_Address);
      Wire.write(Lidar_3_str.c_str());
      Wire.endTransmission();
    }

     if (Lidar4_newDistance)
    {

      //=========================================================================================
      // This comment block can be used to get the Lidar data when running the file in solo mode
      //=========================================================================================
      /*Lidar_4_Data = SD.open("Lidar_4_Data.txt", FILE_WRITE);

      //Send the data from the lidar to the text file above
      if (Lidar_4_Data)
      {
        Lidar_4_Data.print("Lidar 4 distance is: "); Lidar_4_Data.print(Lidar4_newDistance); Lidar_4_Data.println("cm");
        
        //Close the SD Card
        Lidar_4_Data.close();
      }*/
      
      String Lidar_4_str = "Lidar 4 distance is: ";
      Lidar_4_str.append(Lidar4_newDistance);

      Wire.beginTransmission(Other_Address);
      Wire.write(Lidar_4_str.c_str());
      Wire.endTransmission();
    }
  }
}

//*************************************************************************************************************************************************************************

//---------------------------------------------------------------------
// Read Continuous Distance Measurements
//
// The most recent distance measurement can always be read from
// device registers. Polling for the BUSY flag in the STATUS
// register can alert the user that the distance measurement is new
// and that the next measurement can be initiated. If the device is
// BUSY this function does nothing and returns 0. If the device is
// NOT BUSY this function triggers the next measurement, reads the
// distance data from the previous measurement, and returns 1.
//---------------------------------------------------------------------
uint8_t Lidar1_distanceContinuous(uint16_t * distance)
{
  uint8_t newDistance = 0;

  // Check on busyFlag to indicate if device is idle
  // (meaning = it finished the previously triggered measurement)
  if (Lidar1.getBusyFlag() == 0)
  {
    // Trigger the next range measurement
    Lidar1.takeRange();

    // Read new distance data from device registers
    newDistance = Lidar1.readDistance();

  }

  return newDistance;
}


uint8_t Lidar2_distanceContinuous(uint16_t * distance)
{
  uint8_t newDistance = 0;

  // Check on busyFlag to indicate if device is idle
  // (meaning = it finished the previously triggered measurement)
  if (Lidar2.getBusyFlag() == 0)
  {
    // Trigger the next range measurement
    Lidar2.takeRange();

    // Read new distance data from device registers
    newDistance = Lidar2.readDistance();

  }

  return newDistance;
}

uint8_t Lidar3_distanceContinuous(uint16_t * distance)
{
  uint8_t newDistance = 0;

  // Check on busyFlag to indicate if device is idle
  // (meaning = it finished the previously triggered measurement)
  if (Lidar3.getBusyFlag() == 0)
  {
    // Trigger the next range measurement
    Lidar3.takeRange();

    // Read new distance data from device registers
    newDistance = Lidar3.readDistance();

  }

  return newDistance;
}

uint8_t Lidar4_distanceContinuous(uint16_t * distance)
{
  uint8_t newDistance = 0;

  // Check on busyFlag to indicate if device is idle
  // (meaning = it finished the previously triggered measurement)
  if (Lidar4.getBusyFlag() == 0)
  {
    // Trigger the next range measurement
    Lidar4.takeRange();

    // Read new distance data from device registers
    newDistance = Lidar4.readDistance();

  }

  return newDistance;
}
