/*
In this file are all settings needed for the robot
*/

#pragma once

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include "JAFD/header/AllDatatypes.h"

namespace JAFDSettings
{
	namespace Mechanics
	{
		constexpr float wheelDiameter = 8.0f;
		constexpr float wheelDistance = 15.0f;
	}

	namespace SpiEeprom
	{
		constexpr uint8_t ssPin = 13;
	}

	namespace Controller
	{
		namespace Motor
		{
			constexpr float kp = 0.2f;
			constexpr float ki = 1.0f;
			constexpr float kd = 0.005f;
			constexpr float maxCorVal = 0.35f;
		}

		namespace PurePursuit
		{
			constexpr float lookAheadGain = 0.6f;
			constexpr float minLookAheadDist = 5.0f;
			constexpr float maxCurvature = 0.06f;
		}

		namespace SmoothDriving
		{
			namespace ForwardVel
			{
				constexpr float kp = 0.2f;
				constexpr float ki = 2.0f;
				constexpr float kd = 10.0f;
				constexpr uint8_t maxCorVal = 10;
			}

			namespace AngularVel
			{
				constexpr float kp = 0.1f;
				constexpr float ki = 0.5f;
				constexpr float kd = 1.0f;
				constexpr float maxCorVal = 2.0f;
			}
		}
	}

	namespace MotorControl
	{
		constexpr float cmPSToPerc = 0.0075f;	// Conversion factor from cm/s to motor PWM duty cycle (NOTE: The conversion isnt linear. This factor is too low for very low speeds and too high for maximum speed. It is ideal for about 100cm/s)

		constexpr int16_t minSpeed = 15;		// Minimum speed for motor to rotate

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
			constexpr uint8_t dirPin = A7;	// Direction pin left motor
			constexpr uint8_t fbPin = A10;	// Current feedback output left motor
			constexpr uint8_t encA = 10;	// Encoder Pin A
			constexpr uint8_t encB = 11;	// Encoder Pin B
		}
	}

	namespace SmoothDriving
	{

	}

	namespace Dispenser
	{

	}
}