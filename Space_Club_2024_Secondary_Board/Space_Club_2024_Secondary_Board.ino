/*
This sketch is the secondary board for the WVU Tech 2024 Space Club Project
Most of the function of this sketch is identical to the 2023 project. Additional 
functions relating to handling the LIDAR data are added to this version. These LIDAR
functions are currently incomplete and require further testing/design. This sketch was created
by Ian Baily and Ryan Hayden.

*/
//Comments refer to the line or block below them

#include <Adafruit_Sensor.h>
#include <SD.h>
#include <Wire.h>
#include <WireIMXRT.h>
#include <WireKinetis.h>
#include <L3G.h>
//Gyroscope, 3 axis, records rotation data
#include <Adafruit_L3GD20.h>
#include <Adafruit_L3GD20_U.h>
//Magnetometer, records and measures magnetic fields
#include <Adafruit_MMC56x3.h>
//Accelerometer, 3 axis, records acceleration on all three axis.
#include <Adafruit_ADXL375.h>


//Our magnetometer object
Adafruit_MMC5603 magnet;
//Our Accelerometer object
Adafruit_ADXL375 accel = Adafruit_ADXL375(12345);
//Our Gyroscope object;
Adafruit_L3GD20_Unified gyro = Adafruit_L3GD20_Unified(20);

//Our sd card
int sd = BUILTIN_SDCARD;

//LED pin
const int LightPin =  13;

//The file object for our SD card
File SensorData;

#define This_Address 45
#define Other_Address 30

//***************************************************************************************************************************
void setup()
{

  //Light setup Sequence

  // initialize the digital pin as an output
  pinMode(LightPin, OUTPUT);

  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  //SD Card Setup Sequence

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

  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~`
  //Gyroscope setup

  // Enable auto-ranging
  gyro.enableAutoRange(true);

  // Initialise the sensor
  if (!gyro.begin())
  {
    //Log the initialization error
    SensorData = SD.open("GyroscopeData.txt", FILE_WRITE);

    if (SensorData)
    {
      SensorData.println("Gyroscope failed to initialize");
      SensorData.close();
    }

    //This will make the built-in LED blink twice a second if the gyrosocpe fails to initialize
    while (1)
    {
      digitalWrite(LightPin, HIGH);
      delay(500);
      digitalWrite(LightPin, LOW);
      delay(500);
    }
  }

  //Log the successful initialization
  SensorData = SD.open("GyroscopeData.txt", FILE_WRITE);

  if (SensorData)
  {
    SensorData.println("Gyroscope initialization successful");
    SensorData.close();
  }

  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  //Accelerometer Setup Sequence

  //Initializing the sensor
  if (!accel.begin())
  {

    //Log the initialization error
    SensorData = SD.open("AccelerometerData.txt", FILE_WRITE);

    if (SensorData)
    {
      SensorData.println("Accelerometer failed to initialize");
      SensorData.close();
    }

    //This will cause the built-in LED to blink four times a second if the accelerometer fails to initialize
    while (1)
    {
      digitalWrite(LightPin, HIGH);
      delay(250);
      digitalWrite(LightPin, LOW);
      delay(250);
    }
  }

  //Log the successful initialization
  SensorData = SD.open("AccelerometerData.txt", FILE_WRITE);

  if (SensorData)
  {
    SensorData.println("Accelerometer initialization successful");
    SensorData.close();
  }

  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  //Magnetometer Setup Sequence

  //Initializing the sensor
  if (!magnet.begin(MMC56X3_DEFAULT_ADDRESS, &Wire))
  {

    //Log the initialization failure
    SensorData = SD.open("MagnetometerData.txt", FILE_WRITE);

    if (SensorData)
    {
      SensorData.println("Magnetometer failed to initialize");
      SensorData.close();
    }

    //This will cause the built-in LED to blink 10 times per second if the magnetometer fails to initialize
    while (1)
    {
      digitalWrite(LightPin, HIGH);
      delay(100);
      digitalWrite(LightPin, LOW);
      delay(100);
    }
  }

  //Log the successful initialization
  SensorData = SD.open("MagnetometerData.txt", FILE_WRITE);

  if (SensorData)
  {
    SensorData.println("Magnetometer initialization successful");
    SensorData.close();
  }
}
//*********************************************************************************************************************************************
void loop()
{
  //Set up the sensor events for all three devices

  //Declare a sensor event for the gyroscope
  sensors_event_t eventG;
  gyro.getEvent(&eventG);

  //Declare a sensor event for the accelerometer
  sensors_event_t eventA;
  accel.getEvent(&eventA);

  //Declare a sensor event for the magnetometer
  sensors_event_t eventM;
  magnet.getEvent(&eventM);

  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  //SD Card Gyroscope Loop Sequence

  SensorData = SD.open("GyroscopeData.txt", FILE_WRITE);

  //Send the data from the gyroscope to the text file above
  if (SensorData)
  {
    SensorData.print("X: "); SensorData.print(eventG.gyro.x); SensorData.print("  ");
    SensorData.print("Y: "); SensorData.print(eventG.gyro.y); SensorData.print("  ");
    SensorData.print("Z: "); SensorData.print(eventG.gyro.z); SensorData.print("  ");
    SensorData.print("rad/s ");
    //Delay for 500 miliseconds before reprinting the results
    //delay(500);
    //Close the SD Card
    SensorData.close();

  }

  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  //SD Card Accelerometer Loop Sequence

  SensorData = SD.open("AccelerometerData.txt", FILE_WRITE);

  //Send the data from the accelerometer to the text file above
  if (SensorData)
  {
    SensorData.print("X: "); SensorData.print(eventA.acceleration.x); SensorData.print(" ");
    SensorData.print("Y: "); SensorData.print(eventA.acceleration.y); SensorData.print(" ");
    SensorData.print("Z: "); SensorData.print(eventA.acceleration.z); SensorData.print(" ");
    SensorData.print("m/s^2 ");
    //Delay for 500 miliseconds before reprinting the results
    //delay(500);
    //Close the SD Card
    SensorData.close();
  }

  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  //SD Card Magnetometer Loop Sequence

  SensorData = SD.open("MagnetometerData.txt", FILE_WRITE);

  //Send the data from the magnetometer to the text file above
  if (SensorData)
  {
    SensorData.print("X: "); SensorData.print(eventM.magnetic.x); SensorData.print("  ");
    SensorData.print("Y: "); SensorData.print(eventM.magnetic.y); SensorData.print("  ");
    SensorData.print("Z: "); SensorData.print(eventM.magnetic.z); SensorData.print("  ");
    SensorData.print("uT ");

    //Measure and record the temperature
    float temperature = magnet.readTemperature();
    SensorData.print("Temp: "); SensorData.print(temperature); SensorData.print(" *C ");

    //Delay for 500 miliseconds before reprinting the results
    //delay(500);

    //Close the SD Card
    SensorData.close();
  }
  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  //Calculate the seconds since initialization

  //Log the data in each file
  SensorData = SD.open("GyroscopeData.txt", FILE_WRITE);
  if (SensorData)
  {
    SensorData.print("T: ");
    SensorData.println(millis() / 1000.0);
    SensorData.close();
  }

  SensorData = SD.open("AccelerometerData.txt", FILE_WRITE);
  if (SensorData)
  {
    SensorData.print("T: ");
    SensorData.println(millis() / 1000.0);
    SensorData.close();
  }

  SensorData = SD.open("MagnetometerData.txt", FILE_WRITE);
  if (SensorData)
  {
    SensorData.print("T: ");
    SensorData.println(millis() / 1000.0);
    SensorData.close();
  }


  /*Wire.begin(other_address);
  Wire.onReceive(HandleLIDARData);
  if(flag == false)
  {
    Serial.println("Error in transmitting data"); 
  }*/
  
  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  //This will turn the built-in LED on with no blinking if everything is initialized and data is being written
  digitalWrite(LightPin, HIGH);

}

//This method is a work in progress
void HandleLIDARData(int Bytes)
{
  
  int temp = Wire.read();
  flag == true;
}
