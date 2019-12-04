/*
In this file are all settings needed for the robot
*/

#pragma once

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include <stdint.h>

namespace JAFDSettings
{
	namespace Mechanics
	{
		constexpr float wheelDiameter = 8.0f;
	}

	namespace SpiEeprom
	{
		constexpr uint8_t ssPin = 10;
	}

	namespace MotorControl
	{
		// PID - Values
		constexpr float kp = 0.0f;
		constexpr float ki = 0.0f;
		constexpr float kd = 0.0f;

		constexpr float cmPSToPerc = 60.0f / (Mechanics::wheelDiameter * M_PI * 322.0f);	// Conversion factor from cm/s to motor PWM duty cycle

		namespace Left
		{
			constexpr uint8_t pwmPin = 34;	// PWM pin left motor
			constexpr uint8_t dirPin = A4;	// Direction pin left motor
			constexpr uint8_t fbPin = A11;	// Current feedback output left motor
			constexpr uint8_t encA = 22;	// Encoder Pin A
			constexpr uint8_t encB = 23;	// Encoder Pin B
		}

		namespace Right
		{
			constexpr uint8_t pwmPin = 36;	// PWM pin left motor
			constexpr uint8_t dirPin = A1;	// Direction pin left motor
			constexpr uint8_t fbPin = A10;	// Current feedback output left motor
			constexpr uint8_t encA = 24;	// Encoder Pin A
			constexpr uint8_t encB = 25;	// Encoder Pin B
		}
	}

	namespace Dispenser
	{

	}
}