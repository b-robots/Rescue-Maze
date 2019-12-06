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
#include "JAFD/implementation/MotorControl_private.h"
#include "JAFD/implementation/MazeMapping_private.h"
#include "JAFD/implementation/Interrupts_private.h"

using namespace JAFD::MazeMapping;
using namespace JAFD::MotorControl;

float x = 0.0f;

// The setup function runs once when you press reset or power the board
void setup() {
	// For testing
	Serial.begin(115200);

	// If robot is completely stuck, just do nothing.
	if (JAFD::robotSetup() == JAFD::ReturnCode::fatalError)
	{
		while (true);
	}

	for (float s = 20; s <= 120; s += 1)
	{
		setSpeed(Motor::left, s);
		setSpeed(Motor::right, s);

		delay(100);
	}
}

// The loop function runs over and over again until power down or reset
void loop() {
	// If robot is completely stuck, just do nothing.
	if (JAFD::robotLoop() == JAFD::ReturnCode::fatalError)
	{
		while (true);
	}
}
