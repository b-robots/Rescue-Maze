/*
 Name:		MainProgram.ino
 Created:	07.08.2019 14:16:07
 Author:	B.Robots
*/

// External Libs
#include <SPI.h>
#include <VL53L0X.h>
#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_TCS34725.h>
#include <Adafruit_AMG88xx.h>
#include <TPA81.h>
#include <U8glib.h> 
#include <string> 

// RobotLibrary
#include "JAFD/JAFD.h"
#include "JAFD/header/OLed.h"
using namespace JAFD::OLed;

// The setup function runs once when you press reset or power the board
;
void setup()
{
    Serial.begin(115200);
    
	// For testing
	 //Set font.
}

//0x3C OLED Adress
void loop()
{
    auto aaa = getOLedInstance();

    aaa->firstPage();
    do
    {
        robotStopWatch(1);
    } while (aaa->nextPage());

}