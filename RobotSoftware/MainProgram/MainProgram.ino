/*
 Name:		MainProgram.ino
 Created:	07.08.2019 14:16:07
 Author:	B.Robots
*/

// RobotLibrary
#include <RobotLibrary.h>

// The setup function runs once when you press reset or power the board
void setup() {
	/*
	// Robot Settings
	JAFTD::RobotSettings robotSettings;
	robotSettings.mazeMapperSet.ramSSPin = 0;

	// If robot is completely stuck, just do nothing.
	if (robotSetup(robotSettings) == JAFTD::ReturnCode::fatalError)
	{
		while (true);
	}*/
	Serial.begin(115200);
	Serial.println("test");
	pinMode(A0, OUTPUT);
}


// The loop function runs over and over again until power down or reset
void loop() {
	digitalWrite(A0, HIGH);
	delay(500);
	digitalWrite(A0, LOW);
	delay(500);
	/*// If robot is completely stuck, just do nothing.
	if (JAFTD::robotLoop() == JAFTD::ReturnCode::fatalError)
	{
		while (true);
	}*/
}
