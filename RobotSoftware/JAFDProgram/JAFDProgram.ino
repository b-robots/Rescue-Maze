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

using namespace JAFD::MazeMapping;
using namespace JAFD::MotorControl;
using namespace JAFD::SmoothDriving;
using namespace JAFD::SensorFusion;
using namespace JAFD;

int i = 0;

// The setup function runs once when you press reset or power the board
void setup() {
	// For testing
	Serial.begin(115200);

	JAFD::robotSetup();
}

// The loop function runs over and over again until power down or reset
void loop() {
	static ReturnCode c;
	if (isTaskFinished())
	{
		if (i % 6 == 0)
		{
			c = setNewTask(DriveStraight(50, 200.0f));
		}
		else if (i % 6 == 1)
		{
			c = setNewTask(DriveStraight(100, 300.0f));
		}
		else if (i % 6 == 2)
		{
			c = setNewTask(DriveStraight(0, 300.0f));
		}
		else if (i % 6 == 3)
		{
			c = setNewTask(DriveStraight(-70, -600.0f));
		}
		else if (i % 6 == 4)
		{
			c = setNewTask(DriveStraight(0, -400.0f));
		}
		else if (i % 6 == 5)
		{
			c = setNewTask(DriveStraight(100, 200.0f));
		}

		if (c != ReturnCode::ok)
		{
			Serial.println(i);
		}

		i++;
	}
	
	JAFD::robotLoop();

	delay(100);
}
