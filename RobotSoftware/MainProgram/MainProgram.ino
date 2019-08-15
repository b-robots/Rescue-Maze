/*
 Name:		MainProgram.ino
 Created:	07.08.2019 14:16:07
 Author:	B.Robots
*/

// These Includes are necessary for the RobotLibrary to work
#include <Servo.h>
#include <SpiRAM.h>

// RobotLibrary
#include <RobotLibrary.h>

#define LED_PIN 15

using namespace JAFTD;

// The setup function runs once when you press reset or power the board
void setup() {
	pinMode(LED_PIN, OUTPUT);
	// Robot Settings
	RobotSettings robotSettings;
	robotSettings.mazeMapperSet.ramSSPin = 0;

	// If robot is completely stuck, just do nothing.
	if (robotSetup(robotSettings) == ReturnCode::fatalError)
	{
		while (true);
	}

	// Test the SpiRam
	MazeMapping::GridCell cell;
	MazeMapping::MapCoordinate coor;
	uint16_t value;
	for (uint8_t floor = 0; floor <= 1; floor++)
	{
		for (int8_t x = -32; x < 32; x++)
		{
			for (int8_t y = -32; y < 32; y)
			{
				value++;
				cell.cellConnections = value & 0b11111111;
				cell.cellState = value >> 8;
				coor = { x, y, floor };
				MazeMapping::setGridCell(cell, coor);
			}
		}
	}
	MazeMapping::BFAlgorithm::findShortestPath({ 1, 1, 0}, { 0, 0, 0 }, nullptr, 0);
	value = 0;
	for (uint8_t floor = 0; floor <= 1; floor++)
	{
		for (int8_t x = -32; x < 32; x++)
		{
			for (int8_t y = -32; y < 32; y)
			{
				value++;
				coor = { x, y, floor };
				MazeMapping::getGridCell(&cell, coor);
				if (cell.cellConnections != value & 0b11111111 || cell.cellState != value >> 8)
				{
					digitalWrite(LED_PIN, HIGH);
				}
			}
		}
	}
}

// The loop function runs over and over again until power down or reset
void loop() {

	// If robot is completely stuck, just do nothing.
	if (robotLoop() == ReturnCode::fatalError)
	{
		while (true);
	}
}
