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
#include "JAFD/header/MotorControl.h"
#include "JAFD/header/MazeMapping.h"
#include "JAFD/header/Interrupts.h"
#include "JAFD/header/SmoothDriving.h"
#include "JAFD/header/AllDatatypes.h"
#include "JAFD/header/SensorFusion.h"
#include "JAFD/header/DistanceSensors.h"
#include "JAFD/header/Bno055.h"
#include "JAFD/header/SpiNVSRAM.h"

// The setup function runs once when you press reset or power the board
void setup()
{
	// For testing
	Serial.begin(115200);

	JAFD::robotSetup();

	delay(100);
}

// The loop function runs over and over again until power down or reset
void loop()
{
	JAFD::robotLoop();
}
