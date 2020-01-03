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
			constexpr float kp = 0.3f;
			constexpr float ki = 1.2f;
			constexpr float kd = 0.005f;
			constexpr float maxCorVal = 0.35f;
		}

		namespace PathFollowing
		{
			constexpr float lookAheadGain = 0.5f;
			constexpr float minLookAheadDist = 5.0f;
			constexpr float maxCurvature = 0.06f;
			constexpr float kp = 0.0f;
			constexpr float ki = 0.0f;
			constexpr float kd = 0.0f;
		}
	}

	namespace MotorControl
	{
		constexpr float cmPSToPerc = 0.009f;	// Conversion factor from cm/s to motor PWM duty cycle

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
		namespace Accelerate
		{
			// PID - Values
			constexpr float kp = 0.002f;
			constexpr float ki = 0.01f;
			constexpr float kd = 0.001f;
		}

		namespace DriveStraight
		{
			// PID - Values
			constexpr float kp = 0.002f;
			constexpr float ki = 0.01f;
			constexpr float kd = 0.001f;
		}
	}

	namespace Dispenser
	{

	}
}