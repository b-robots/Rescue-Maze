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
#include <utility/DuePinMapping_public.h>

// The setup function runs once when you press reset or power the board
void setup() {
	// Robot Settings
	JAFD::RobotSettings robotSettings;
	robotSettings.mazeMapperSet.ramSSPin = A1;

	PMC->PMC_PCER0 = 1 << JAFD::PinMapping::MappedPins[A0].portID;
	JAFD::PinMapping::MappedPins[A0].port->PIO_PER = JAFD::PinMapping::MappedPins[A0].pin;
	JAFD::PinMapping::MappedPins[A0].port->PIO_OER = JAFD::PinMapping::MappedPins[A0].pin;

	// If robot is completely stuck, just do nothing.
	if (robotSetup(robotSettings) == JAFD::ReturnCode::fatalError)
	{
		while (true);
	}
}

// The loop function runs over and over again until power down or reset
void loop() {
	JAFD::PinMapping::MappedPins[A0].port->PIO_SODR = JAFD::PinMapping::MappedPins[A0].pin;
	delay(500);
	JAFD::PinMapping::MappedPins[A0].port->PIO_CODR = JAFD::PinMapping::MappedPins[A0].pin;
	delay(500);

	// If robot is completely stuck, just do nothing.
	if (JAFD::robotLoop() == JAFD::ReturnCode::fatalError)
	{
		while (true);
	}
}
