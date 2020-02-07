/*
 Name:		MainProgram.ino
 Created:	07.08.2019 14:16:07
 Author:	B.Robots
*/

#if defined(ARDUINO) && ARDUINO >= 100
#include <SPI.h>
#include <Adafruit_VL53L0X.h>	// Adafruit library
#include <Wire.h>
#include "arduino.h"
#else
#include "WProgram.h"
#endif

// RobotLibrary
#include "JAFD/JAFD.h"
#include "JAFD/header/MotorControl.h"
#include "JAFD/header/MazeMapping.h"
#include "JAFD/header/Interrupts.h"
#include "JAFD/header/SmoothDriving.h"
#include "JAFD/header/AllDatatypes.h"
#include "JAFD/header/SensorFusion.h"
#include "JAFD/header/DistanceSensors.h"

using namespace JAFD::MazeMapping;
using namespace JAFD::MotorControl;
using namespace JAFD::SmoothDriving;
using namespace JAFD::SensorFusion;
using namespace JAFD::DistanceSensors;
using namespace JAFD;

// The setup function runs once when you press reset or power the board
void setup()
{
	// For testing
	Serial.begin(115200);

	JAFD::robotSetup();
}

// The loop function runs over and over again until power down or reset
void loop()
{
	static uint16_t frontDist = 0;

	frontDist = frontNew.getDistance();

	Serial.println(frontNew.getStatus() == VL53L0::Status::noError ?  frontDist : "Error: " + String((int)frontNew.getStatus()));

	JAFD::robotLoop();

	delay(100);
}
