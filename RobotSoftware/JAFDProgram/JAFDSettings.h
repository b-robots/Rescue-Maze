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
#include "JAFD/header/PIDController.h"

namespace JAFDSettings
{
	namespace Field
	{
		constexpr float cellWidth = 30.0f;	// Cell width in cm
	}

	namespace Mechanics
	{
		constexpr float wheelDiameter = 8.0f;
		constexpr float wheelDistance = 15.5f;
		constexpr float sensorLeftRightDist = 17.0f;
		constexpr float sensorFrontBackDist = 16.0f;
	}

	namespace SpiNVSRAM
	{
		constexpr uint8_t ssPin = 13;
	}

	namespace MotorControl
	{
		constexpr float cmPSToPerc = 1.0f / (97.0f / 60.0f * 3.1415f * Mechanics::wheelDiameter);		// Conversion factor from cm/s to motor PWM duty cycle (NOTE: The conversion isnt linear. This factor is too low for very low speeds and too high for maximum speed. It is ideal for about 100cm/s)

		constexpr int16_t minSpeed = 8;			// Minimum speed for motor to rotate

		constexpr float pulsePerRev = 4741.44f / 4.0f;		// Rotary-Encoder pulses per revolution

		namespace Left
		{
			constexpr uint8_t pwmPin = 34;	// PWM pin left motor
			constexpr uint8_t dirPin = 53;	// Direction pin left motor
			constexpr uint8_t fbPin = A11;	// Current feedback output left motor
			constexpr uint8_t encA = 23;	// Encoder Pin A
			constexpr uint8_t encB = 22;	// Encoder Pin B
		}

		namespace Right
		{
			constexpr uint8_t pwmPin = 36;	// PWM pin left motor
			constexpr uint8_t dirPin = A7;	// Direction pin left motor
			constexpr uint8_t fbPin = A10;	// Current feedback output left motor
			constexpr uint8_t encA = 11;	// Encoder Pin A
			constexpr uint8_t encB = 10;	// Encoder Pin B
		}
	}

	namespace Controller
	{
		namespace Motor
		{
			constexpr JAFD::PIDSettings pidSettings(1.0f, 5.5f, 0.01f, 1.0f / MotorControl::cmPSToPerc, 0.5f / MotorControl::cmPSToPerc, -1.0f / MotorControl::cmPSToPerc, 1.0f / MotorControl::cmPSToPerc);
		}

		namespace PurePursuit
		{
			constexpr float lookAheadGain = 0.9f;
			constexpr float minLookAheadDist = 12.0f;
			constexpr float maxCurvature = 0.02f;
		}

		namespace SmoothDriving
		{
			constexpr JAFD::PIDSettings forwardVelPidSettings(0.3f, 1.5f, 0.0f, 1.0f / MotorControl::cmPSToPerc, 0.5f / MotorControl::cmPSToPerc, -1.0f / MotorControl::cmPSToPerc, 1.0f / MotorControl::cmPSToPerc);
			constexpr JAFD::PIDSettings angularVelPidSettings(0.3f, 1.3f, 0.0f, 10.0f, 5.0f, -10.0f, 10.0f);
		}
	}

	namespace SmoothDriving
	{
		constexpr uint8_t maxArrrayedTasks = 5;
	}

	namespace Dispenser
	{

	}

	namespace DistanceSensors
	{
		namespace FrontLong
		{
			constexpr JAFD::SerialType serialType = JAFD::SerialType::three;
		}

		namespace BackLong
		{
			constexpr JAFD::SerialType serialType = JAFD::SerialType::one;
		}
	}
}