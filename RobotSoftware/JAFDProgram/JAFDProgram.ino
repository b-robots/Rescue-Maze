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

// The setup function runs once when you press reset or power the board
void setup() {
	// For testing
	Serial.begin(115200);

	// If robot is completely stuck, just do nothing.
	if (JAFD::robotSetup() == JAFD::ReturnCode::fatalError)
	{
		while (true);
	}

	setSpeed(Motor::left, 1.0f);
	setSpeed(Motor::right, 0.5f);
}

// The loop function runs over and over again until power down or reset
void loop() {
	// If robot is completely stuck, just do nothing.
	if (JAFD::robotLoop() == JAFD::ReturnCode::fatalError)
	{
		while (true);
	}

	delay(500);

	Serial.print(getVelocity(Motor::left));
	Serial.print(", ");
	Serial.println(getDistance(Motor::left));
}
