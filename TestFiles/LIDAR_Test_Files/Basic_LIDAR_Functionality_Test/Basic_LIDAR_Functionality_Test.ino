/* Basic LIDAR functionality test file
 * 
 * This file can be used to test each of the range settings 
 * built into the lidar. They will preform the appropriate type
 * of scan and then display the distance results in the serial monitor. 
 * This type of test requires access to the serial monitor and requires 
 * a character input to function. For our actual program this can be
 * changed so we dont need to do that. Ive gone through and commented the sample
 * code and tried to add context to what the code is doing. Ive also left any relevant
 * wiring/electrical information provided in the sample code in the file. I'll use asteriks
 * separate any functions to make it easier to see what is where
 */
 
//These are the libraries need
#include <SD.h>
#include <stdint.h>
#include <Wire.h>
#include "LIDARLite_v4LED.h"

//Our lidar object
LIDARLite_v4LED myLidarLite;

#define FAST_I2C

int sd = BUILTIN_SDCARD;

File LIDAR_Functionality_Test_Data;

//define which pins are the monitor and trigger pins
#define MonitorPin    3
#define TriggerPin    2

//This enum datatype is used to hold all the range types simultaneously
//The GPIO functions work the same but use the GPIO pin on the LIDAR to function
enum rangeType_T
{
  RANGE_NONE,
  RANGE_SINGLE,
  RANGE_CONTINUOUS,
  RANGE_SINGLE_GPIO,
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


LIDAR_Functionality_Test_Data = SD.open("Lidar_1_Data.txt", FILE_WRITE);

  if (LIDAR_Functionality_Test_Data)
  {
    LIDAR_Functionality_Test_Data.println("Functionality Test File Successfully Initialized");
    LIDAR_Functionality_Test_Data.close();
  }

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
  myLidarLite.configure(0);

}
//======================
//loop function
//======================
//***********************************************************************************************************************************************
void loop()
{
  //despite being in the loop function this little block will only be triggered once
  uint16_t distance;
  uint8_t  newDistance;
  uint8_t  inputChar;
  rangeType_T rangeMode = RANGE_NONE;

  //MenuPrint();

  //Thats because of this continuous loop
  while (1)
  {
    //===================================================================
    // 1) Each time through the loop, look for a serial input character
    //===================================================================
    
      //  read input character ...
      //inputChar = (uint8_t) 2;

        rangeMode = RANGE_CONTINUOUS;
    

    //===================================================================
    // 2) Check on mode and operate accordingly
    //===================================================================
    switch (rangeMode)
    {
      case RANGE_NONE:
        newDistance = 0;
        break;

      case RANGE_SINGLE:
        newDistance = distanceSingle(&distance);
        rangeMode   = RANGE_NONE;
        break;

      case RANGE_CONTINUOUS:
        newDistance = distanceContinuous(&distance);
        break;

      case RANGE_SINGLE_GPIO:
        newDistance = distanceSingleGpio(&distance);
        rangeMode   = RANGE_NONE;
        break;

      case RANGE_CONTINUOUS_GPIO:
        newDistance = distanceContinuousGpio(&distance);
        break;

      default:
        newDistance = 0;
        rangeMode   = RANGE_NONE;
        break;
    }

    //===================================================================
    // 3) When there is new distance data, print it to the serial port
    //===================================================================
    if (newDistance)
    {
      Serial.print("Lidar distance is: "); Serial.print(distance);
      
      LIDAR_Functionality_Test_Data = SD.open("Lidar_Functionality_Test_Data.txt", FILE_WRITE);

      //Send the data from the lidar to the text file above
      if (LIDAR_Functionality_Test_Data)
      {
        LIDAR_Functionality_Test_Data.print("Lidar distance is: "); LIDAR_Functionality_Test_Data.print(distance);
        
        //Close the SD Card
        LIDAR_Functionality_Test_Data.close();
      }
    }
  }
}

//---------------------------------------------------------------------
// Menu Print
//---------------------------------------------------------------------
//***********************************************************************************************************************************************
void MenuPrint(void)
{
  //This is the menu that you will see at the begining of running the program
  Serial.println("");
  Serial.println("============================================");
  Serial.println("== LLv4 - Type a single character command ==");
  Serial.println("============================================");
  Serial.println(" 1 - Single Measurement");
  Serial.println(" 2 - Continuous Measurement");
  Serial.println(" 3 - Single Measurement using trigger / monitor pins");
  Serial.println(" 4 - Continuous Measurement using trigger / monitor pins");
  Serial.println(" 5 - Dump Correlation Record");
  Serial.println(" . - Stop Measurement");
  Serial.println(" V - Print Version Numbers");
  Serial.println("");
}

//---------------------------------------------------------------------
// Read Single Distance Measurement
//
// This is the simplest form of taking a measurement. This is a
// blocking function as it will not return until a range has been
// taken and a new distance measurement can be read.
//---------------------------------------------------------------------
//***********************************************************************************************************************************************
uint8_t distanceSingle(uint16_t * distance)
{
  // 1. Trigger range measurement.
  myLidarLite.takeRange();

  // 2. Wait for busyFlag to indicate device is idle.
  myLidarLite.waitForBusy();

  // 3. Read new distance data from device registers
  *distance = myLidarLite.readDistance();

  return 1;
}

//---------------------------------------------------------------------
// Read Single Distance Measurement using Trigger / Monitor Pins
//
// This is the simplest form of taking a measurement. This is a
// blocking function as it will not return until a range has been
// taken and a new distance measurement can be read. Instead of using
// the STATUS register to poll for BUSY, this function uses a
// GPIO pin on the LIDAR-Lite to monitor the BUSY flag.
//---------------------------------------------------------------------
//***********************************************************************************************************************************************
uint8_t distanceSingleGpio(uint16_t * distance)
{
  // 1. Trigger range measurement.
  myLidarLite.takeRangeGpio(TriggerPin, MonitorPin);

  // 2. Wait for busyFlag to indicate device is idle.
  myLidarLite.waitForBusyGpio(MonitorPin);

  // 3. Read new distance data from device registers
  *distance = myLidarLite.readDistance();

  return 1;
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
uint8_t distanceContinuous(uint16_t * distance)
{
  uint8_t newDistance = 0;

  // Check on busyFlag to indicate if device is idle
  // (meaning = it finished the previously triggered measurement)
  if (myLidarLite.getBusyFlag() == 0)
  {
    // Trigger the next range measurement
    myLidarLite.takeRange();

    // Read new distance data from device registers
    *distance = myLidarLite.readDistance();

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
uint8_t distanceContinuousGpio(uint16_t * distance)
{
  uint8_t newDistance = 0;

  // Check on busyFlag to indicate if device is idle
  // (meaning = it finished the previously triggered measurement)
  if (myLidarLite.getBusyFlagGpio(MonitorPin) == 0)
  {
    // Trigger the next range measurement
    myLidarLite.takeRangeGpio(TriggerPin, MonitorPin);

    // Read new distance data from device registers
    *distance = myLidarLite.readDistance();

    // Report to calling function that we have new data
    newDistance = 1;
  }

  return newDistance;
}

//---------------------------------------------------------------------
// Print the correlation record for analysis
//---------------------------------------------------------------------
//***********************************************************************************************************************************************
void dumpCorrelationRecord()
{
    int16_t corrValues[192];
    uint8_t i;

    myLidarLite.correlationRecordRead(corrValues);

    for (i=0 ; i<192 ; i++)
    {
        Serial.print(corrValues[i], DEC);
        Serial.print(",");
    }
    Serial.println(" ");
}

//---------------------------------------------------------------------
void VersionPrint(void)
//---------------------------------------------------------------------
//***********************************************************************************************************************************************
{
    uint8_t    dataBytes[12];
    uint8_t  * nrfVerString;
    uint16_t * lrfVersion;
    uint8_t  * hwVersion;
    uint8_t  i;

    //===========================================
    // Print nRF Software Version
    //===========================================
    myLidarLite.read(0x30, dataBytes, 11, 0x62);
    nrfVerString = dataBytes;
    Serial.print("nRF Software Version  - ");
    for (i=0 ; i<11 ; i++)
    {
      Serial.write(nrfVerString[i]);
    }
    Serial.println("");

    //===========================================
    // Print LRF Firmware Version
    //===========================================
    myLidarLite.read(0x72, dataBytes, 2, 0x62);
    lrfVersion = (uint16_t *) dataBytes;
    Serial.print("LRF Firmware Version  - v");
    Serial.print((*lrfVersion) / 100);
    Serial.print(".");
    Serial.print((*lrfVersion) % 100);
    Serial.println("");

    //===========================================
    // Print Hardware Version
    //===========================================
    myLidarLite.read(0xE1, dataBytes, 1, 0x62);
    hwVersion = dataBytes;
    Serial.print("Hardware Version      - ");
    switch (*hwVersion)
    {
        case 16 : Serial.println("RevA"); break;
        case  8 : Serial.println("RevB"); break;
        default : Serial.println("????"); break;
    }
}
