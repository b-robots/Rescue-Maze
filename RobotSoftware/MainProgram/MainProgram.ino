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
#include <JAFDLibrary.h>
#include <implementation/MazeMapping_private.h>

// The setup function runs once when you press reset or power the board
void setup() {
	// Robot Settings
	JAFD::RobotSettings robotSettings;
	robotSettings.mazeMapperSet.ramSSPin = A1;

	Serial.begin(115200);
	Serial.println("Hallo");

	// If robot is completely stuck, just do nothing.
	if (robotSetup(robotSettings) == JAFD::ReturnCode::fatalError)
	{
		while (true);
	}

	// Test the speed of setting 1000 GridCells
	auto a = micros();
	for (int i = 0; i < 1000; i++)
	{
		JAFD::MazeMapping::setGridCell({ i, JAFD::MazeMapping::CellState::checkpoint }, { JAFD::MazeMapping::SolverState::east, 10 }, { 0, 0, 0 });
	}
	auto time = micros() - a;
}

// The loop function runs over and over again until power down or reset
void loop() {
	// If robot is completely stuck, just do nothing.
	if (JAFD::robotLoop() == JAFD::ReturnCode::fatalError)
	{
		while (true);
	}
}
