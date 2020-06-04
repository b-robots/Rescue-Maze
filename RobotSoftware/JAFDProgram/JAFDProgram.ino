/*
 Name:		MainProgram.ino
 Created:	07.08.2019 14:16:07
 Author:	B.Robots
*/

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

// External Libs
#include <SPI.h>
#include <VL53L0X.h>
#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>

// RobotLibrary
#include "JAFD/JAFD.h"
#include "JAFD/header/Dispenser.h"

// The setup function runs once when you press reset or power the board
void setup()
{
	// For testing
	Serial.begin(115200);

	JAFD::robotSetup();

	delay(100);

	JAFD::Dispenser::dispenseLeft(2);
	JAFD::Dispenser::dispenseRight(2);
}

// The loop function runs over and over again until power down or reset
void loop()
{
	JAFD::robotLoop();
}
