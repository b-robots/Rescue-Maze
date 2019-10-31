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

#include <utility/SpiEeprom_private.h>

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

}

// The loop function runs over and over again until power down or reset
void loop() {
	// If robot is completely stuck, just do nothing.
	if (JAFD::robotLoop() == JAFD::ReturnCode::fatalError)
	{
		while (true);
	}
}
