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
	front.updateValues();
	left.updateValues();

	Serial.print(front.getStatus() == Status::noError ? front.getDistance() : -1.0f);
	Serial.print(", ");
	Serial.println(left.getStatus() == Status::noError ? left.getDistance() : -1.0f);

	JAFD::robotLoop();

	delay(500);
}
