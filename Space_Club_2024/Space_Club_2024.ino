/* 
 *  2024 Space Club Primary Board 
 *  Created by Ian Bailey and Ryan Hayden for the WVU Tech 2024 Space Club project
 *  This version of the file should handle running the lidar in continuous mode without additional
 *  input in the serial monitor and it should properly save data into a files on the built-in SD card.
 *  This file should handle all 4 Lidar modules but additiona testing may be required
*/

//These are the libraries need
#include <SD.h>
#include <stdint.h>
#include <Wire.h>
#include "LIDARLite_v4LED.h"

#define This_Address 30
#define Other_Address 45

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

//Initialize the data file
File LidarData;

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
//***********************************************************************************************************************************************
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

//Define the SDL/SDA pins for the main teensy board
Wire.setSDA(18);
Wire.setSCL(19);

// ----------------------------------------------------------------------
// Lights and SD card setup
// ----------------------------------------------------------------------
//***********************************************************************************************************************************************
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

  LidarData = SD.open("LidarData.txt", FILE_WRITE);

  if (LidarData)
  {
    LidarData.println("LidarFile failed to initialize");
    LidarData.close();
  }

// ----------------------------------------------------------------------
// Additional Setup/Info
// ----------------------------------------------------------------------
//***********************************************************************************************************************************************

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

}
//======================
//loop function
//======================
//***********************************************************************************************************************************************
void loop()
{
  //despite being in the loop function this block will only be triggered once due to the while loop down below
  uint16_t distance;
  uint8_t  Lidar1_newDistance;
  uint8_t  Lidar2_newDistance;
  uint8_t  Lidar3_newDistance;
  uint8_t  Lidar4_newDistance;
  uint8_t  inputChar;
  rangeType_T rangeMode = RANGE_NONE;

  //===================================================================
  // 1) Look for a serial input character to establish RANGE_MODE
  //===================================================================
  if (Serial.available())
  {
    //  read input character ...
    Serial.print("Enter 1 for continuous mode or 2 for continuous GPIO mode");
    inputChar = (uint8_t) Serial.read();

    // ... and parse
    switch (inputChar)
    {

      case '1':
        rangeMode = RANGE_CONTINUOUS;
        break;

      case '2':
        rangeMode = RANGE_CONTINUOUS_GPIO;
        break;

      case '3':
        rangeMode = RANGE_NONE;
        break;

      case '.':
        rangeMode = RANGE_NONE;
        break;

      case ' ':
      case 0x0D:
      case 0x0A:
        rangeMode = RANGE_CONTINUOUS;
        break;

      default:
        rangeMode = RANGE_NONE;
        break;
    }
  }

  //This is where the actual looping occurs
  while (1)
  {

    //===================================================================
    // 2) Check on mode and operate accordingly
    //===================================================================
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

      case RANGE_CONTINUOUS_GPIO:
        Lidar1_newDistance = Lidar1_distanceContinuousGpio(&distance);
        Lidar2_newDistance = Lidar2_distanceContinuousGpio(&distance);
        Lidar3_newDistance = Lidar3_distanceContinuousGpio(&distance);
        Lidar4_newDistance = Lidar4_distanceContinuousGpio(&distance);
        break;

      default:
        Lidar1_newDistance = 0;
        Lidar2_newDistance = 0;
        Lidar3_newDistance = 0;
        Lidar4_newDistance = 0;
        rangeMode   = RANGE_NONE;
        break;
    }

    //===================================================================
    // 3) When there is new distance data, print it to file on the SD card
    //===================================================================
    if (Lidar1_newDistance)
    {
      LidarData = SD.open("Lidar_1_Data.txt", FILE_WRITE);

      //Send the data from the lidar to the text file above
      if (LidarData)
      {
        LidarData.print("Lidar 1 distance is: "); LidarData.print(distance);
        
        //Close the SD Card
        LidarData.close();
      }
    }

     if (Lidar2_newDistance)
    {
      LidarData = SD.open("Lidar_2_Data.txt", FILE_WRITE);

      //Send the data from the lidar to the text file above
      if (LidarData)
      {
        LidarData.print("Lidar 2 distance is: "); LidarData.print(distance);
        
        //Close the SD Card
        LidarData.close();
      }
    }

     if (Lidar3_newDistance)
    {
      LidarData = SD.open("Lidar_3_Data.txt", FILE_WRITE);

      //Send the data from the lidar to the text file above
      if (LidarData)
      {
        LidarData.print("Lidar 3 distance is: "); LidarData.print(distance);
        
        //Close the SD Card
        LidarData.close();
      }
    }

     if (Lidar4_newDistance)
    {
      LidarData = SD.open("Lidar_4_Data.txt", FILE_WRITE);

      //Send the data from the lidar to the text file above
      if (LidarData)
      {
        LidarData.print("Lidar 4 distance is: "); LidarData.print(distance);
        
        //Close the SD Card
        LidarData.close();
      }

      /*Wire.beginTransmission(Other_Address);
        Wire.write(newDistance);
        Wire.endTransmission();*/
    }
  }
}

//---------------------------------------------------------------------
// Menu Print Function
//---------------------------------------------------------------------
//***********************************************************************************************************************************************
void MenuPrint(void)
{
  //This is the menu that you will see at the begining of running the program
  Serial.println("");
  Serial.println("============================================");
  Serial.println("== LLv4 - Type a single character command ==");
  Serial.println("============================================");
  Serial.println(" 1 - Continuous Measurement");
  Serial.println(" 2 - Continuous Measurement using trigger / monitor pins");
  Serial.println(" 3 - Dump Correlation Record");
  Serial.println(" . - Stop Measurement");
  Serial.println("");
}



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
//***********************************************************************************************************************************************
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
    *distance = Lidar1.readDistance();

    // Report to calling function that we have new data
    newDistance = 1;
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
    *distance = Lidar2.readDistance();

    // Report to calling function that we have new data
    newDistance = 1;
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
    *distance = Lidar3.readDistance();

    // Report to calling function that we have new data
    newDistance = 1;
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
    *distance = Lidar4.readDistance();

    // Report to calling function that we have new data
    newDistance = 1;
  }

  return newDistance;
}

//---------------------------------------------------------------------
// Read Continuous Distance Measurements using Trigger / Monitor Pins
//
// The most recent distance measurement can always be read from
// device registers. Polling for the BUSY flag using the Monitor Pin
// can alert the user that the distance measurement is new
// and that the next measurement can be initiated. If the device is
// BUSY this function does nothing and returns 0. If the device is
// NOT BUSY this function triggers the next measurement, reads the
// distance data from the previous measurement, and returns 1.
//---------------------------------------------------------------------
//***********************************************************************************************************************************************
uint8_t Lidar1_distanceContinuousGpio(uint16_t * distance)
{
  uint8_t newDistance = 0;

  // Check on busyFlag to indicate if device is idle
  // (meaning = it finished the previously triggered measurement)
  if (Lidar1.getBusyFlagGpio(MonitorPin) == 0)
  {
    // Trigger the next range measurement
    Lidar1.takeRangeGpio(TriggerPin, MonitorPin);

    // Read new distance data from device registers
    *distance = Lidar1.readDistance();

    // Report to calling function that we have new data
    newDistance = 1;
  }

  return newDistance;
}


uint8_t Lidar2_distanceContinuousGpio(uint16_t * distance)
{
  uint8_t newDistance = 0;

  // Check on busyFlag to indicate if device is idle
  // (meaning = it finished the previously triggered measurement)
  if (Lidar2.getBusyFlagGpio(MonitorPin) == 0)
  {
    // Trigger the next range measurement
    Lidar2.takeRangeGpio(TriggerPin, MonitorPin);

    // Read new distance data from device registers
    *distance = Lidar2.readDistance();

    // Report to calling function that we have new data
    newDistance = 1;
  }

  return newDistance;
}

uint8_t Lidar3_distanceContinuousGpio(uint16_t * distance)
{
  uint8_t newDistance = 0;

  // Check on busyFlag to indicate if device is idle
  // (meaning = it finished the previously triggered measurement)
  if (Lidar3.getBusyFlagGpio(MonitorPin) == 0)
  {
    // Trigger the next range measurement
    Lidar3.takeRangeGpio(TriggerPin, MonitorPin);

    // Read new distance data from device registers
    *distance = Lidar3.readDistance();

    // Report to calling function that we have new data
    newDistance = 1;
  }

  return newDistance;
}

uint8_t Lidar4_distanceContinuousGpio(uint16_t * distance)
{
  uint8_t newDistance = 0;

  // Check on busyFlag to indicate if device is idle
  // (meaning = it finished the previously triggered measurement)
  if (Lidar4.getBusyFlagGpio(MonitorPin) == 0)
  {
    // Trigger the next range measurement
    Lidar4.takeRangeGpio(TriggerPin, MonitorPin);

    // Read new distance data from device registers
    *distance = Lidar4.readDistance();

    // Report to calling function that we have new data
    newDistance = 1;
  }

  return newDistance;
}
