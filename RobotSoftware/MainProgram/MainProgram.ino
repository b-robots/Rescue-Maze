/*
 Name:		MainProgram.ino
 Created:	07.08.2019 14:16:07
 Author:	B.Robots
*/

#include "RobotLibrary.h"

// The setup function runs once when you press reset or power the board
void setup() {

	// If robot is completely stuck, just do nothing.
	if (JAFID::robotSetup() == JAFID::ReturnState::fatalError)
	{
		while (true);
	}
}

// The loop function runs over and over again until power down or reset
void loop() {

	// If robot is completely stuck, just do nothing.
	if (JAFID::robotSetup() == JAFID::ReturnState::fatalError)
	{
		while (true);
	}
}
