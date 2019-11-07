/*
In this file are all settings needed for the robot
*/

#pragma once

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

namespace JAFDSettings
{
	namespace SpiEeprom
	{
		constexpr uint8_t ssPin = 10;
	}

	namespace MotorControl
	{
		namespace Left
		{
			constexpr uint8_t pwmPin = 34;	// PWM pin left motor
			constexpr uint8_t dirPin = A4;	// Direction pin left motor
			constexpr uint8_t fbPin = A11;	// Current feedback output left motor
		}

		namespace Right
		{
			constexpr uint8_t pwmPin = 36;	// PWM pin left motor
			constexpr uint8_t dirPin = A1;	// Direction pin left motor
			constexpr uint8_t fbPin = A10;	// Current feedback output left motor
		}
	}

	namespace Dispenser
	{

	}
}