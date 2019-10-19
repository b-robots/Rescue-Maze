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

using namespace JAFD::MazeMapping;

// The setup function runs once when you press reset or power the board
void setup() {
	// For testing
	Serial.begin(115200);

	// Robot Settings
	JAFD::RobotSettings robotSettings;
	robotSettings.mazeMapperSet.ramSSPin = A1;

	robotSettings.motorControlSet.m1Dir = A11;
	robotSettings.motorControlSet.m1Fb = DAC1;
	robotSettings.motorControlSet.m1PWM = 34;

	robotSettings.motorControlSet.m2Dir = A10;
	robotSettings.motorControlSet.m2Fb = DAC0;
	robotSettings.motorControlSet.m2PWM = 36;

	// If robot is completely stuck, just do nothing.
	if (robotSetup(robotSettings) == JAFD::ReturnCode::fatalError)
	{
		while (true);
	}

	// TODO: Improve read/write Stream
	// Test write / read of Maze Mapping
	// Create test maze
	setGridCell({ 0b1111, CellState::visited | CellState::checkpoint }, homePosition);
	setGridCell({ 0b1101, CellState::visited | CellState::checkpoint }, { homePosition.x + 1, homePosition.y, 0 });
	setGridCell({ 0b1100, CellState::visited | CellState::checkpoint }, { homePosition.x + 1, homePosition.y + 1, 0 });
	setGridCell({ 0b1110, CellState::visited | CellState::checkpoint }, { homePosition.x, homePosition.y + 1, 0 });
	setGridCell({ 0b0110, CellState::visited | CellState::checkpoint }, { homePosition.x - 1, homePosition.y + 1, 0 });
	setGridCell({ 0b0111, CellState::visited | CellState::checkpoint }, { homePosition.x - 1, homePosition.y, 0 });
	setGridCell({ 0b0011, CellState::visited | CellState::checkpoint }, { homePosition.x - 1, homePosition.y - 1, 0 });
	setGridCell({ 0b1011, CellState::visited | CellState::checkpoint }, { homePosition.x, homePosition.y - 1, 0 });
	setGridCell({ 0b1001, CellState::visited | CellState::checkpoint }, { homePosition.x + 1, homePosition.y - 1, 0 });


	GridCell val;
	getGridCell(&val, homePosition);
	Serial.println(val);
}

// The loop function runs over and over again until power down or reset
void loop() {
	// If robot is completely stuck, just do nothing.
	if (JAFD::robotLoop() == JAFD::ReturnCode::fatalError)
	{
		while (true);
	}
}
