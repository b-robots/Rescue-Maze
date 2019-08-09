/*
 Name:		MainProgram.ino
 Created:	07.08.2019 14:16:07
 Author:	B.Robots
*/

// These Includes are necessary for the RobotLibrary to work
#include <SpiRAM.h>

// RobotLibrary
#include "RobotLibrary.h"

// The setup function runs once when you press reset or power the board
void setup() {
	// Robot Settings
	JAFTD::RobotSettings robotSettings;
	robotSettings.mazeMapperSet.ramSSPin = 0;

	// If robot is completely stuck, just do nothing.
	if (JAFTD::robotSetup(robotSettings) == ReturnState::fatalError)
	{
		while (true);
	}
}

// The loop function runs over and over again until power down or reset
void loop() {

	// If robot is completely stuck, just do nothing.
	if (JAFTD::robotLoop() == ReturnState::fatalError)
	{
		while (true);
	}
}
