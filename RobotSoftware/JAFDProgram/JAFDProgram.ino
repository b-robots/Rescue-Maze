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

// The setup function runs once when you press reset or power the board
void setup() {
	// For testing
	Serial.begin(115200);

	JAFD::robotSetup();
}

// The loop function runs over and over again until power down or reset
void loop() {
	static ReturnCode c;
	static int i;
	if (isTaskFinished())
	{
		if (i % 8 == 0)
		{
			c = setNewTask<NewStateType::lastEndState>(Accelerate(100, 100.0f));
		}
		else if (i % 8 == 1)
		{
			c = setNewTask<NewStateType::lastEndState>(DriveStraight(300.0f));
		}
		else if (i % 8 == 2)
		{
			c = setNewTask<NewStateType::lastEndState>(Accelerate(20, 100.0f));
		}
		else if (i % 8 == 3)
		{
			c = setNewTask<NewStateType::lastEndState>(Stop());
		}
		else if (i % 8 == 4)
		{
			c = setNewTask<NewStateType::lastEndState>(Rotate(5.0f, 1000.0f));
		}
		else if (i % 8 == 5)
		{
			c = setNewTask<NewStateType::lastEndState>(Rotate(-5.0f, -1000.0f));
		}
		else if (i % 8 == 6)
		{
			c = setNewTask<NewStateType::lastEndState>(Accelerate(-50, -100.0f));
		}
		else if (i % 8 == 7)
		{
			c = setNewTask<NewStateType::lastEndState>(Stop());
		}

		if (c != ReturnCode::ok)
		{
			Serial.println(i % 8);
		}

		i++;
	}
	//{@Plot.Position.X getRobotState().position.x}{@Plot.Position.Y getRobotState().position.y}{@Plot.Heading.Heading getRobotState().rotation.x * 180.0f / 3.1416f}
	JAFD::robotLoop();

	delay(200);
}
