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
		constexpr float minObstacleSpace = 20.0f;	// Minimum available space between obstacle and border
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
		constexpr uint8_t ssPin = 6;
	}

	namespace MotorControl
	{
		constexpr float cmPSToPerc = 1.0f / (97.0f / 60.0f * 3.1415f * Mechanics::wheelDiameter);		// Conversion factor from cm/s to motor PWM duty cycle (NOTE: The conversion isnt linear. This factor is too low for very low speeds and too high for maximum speed. It is ideal for about 100cm/s)

		constexpr uint8_t minSpeed = 10;			// Minimum speed for motor to rotate

		constexpr float pulsePerRev = 4741.44f / 4.0f;		// Rotary-Encoder pulses per revolution

		namespace Left
		{
			constexpr uint8_t pwmPin = 43;	// PWM pin left motor
			constexpr uint8_t dirPin = 47;	// Direction pin left motor
			constexpr uint8_t fbPin = A11;	// Current feedback output left motor
			constexpr uint8_t encA = 4;		// Encoder Pin A
			constexpr uint8_t encB = 5;		// Encoder Pin B
		}

		namespace Right
		{
			constexpr uint8_t pwmPin = 41;	// PWM pin left motor
			constexpr uint8_t dirPin = 46;	// Direction pin left motor
			constexpr uint8_t fbPin = A10;	// Current feedback output left motor
			constexpr uint8_t encA = 6;		// Encoder Pin A
			constexpr uint8_t encB = 9;		// Encoder Pin B
		}
	}

	namespace Controller
	{
		namespace Motor
		{
			constexpr JAFD::PIDSettings pidSettings(0.85f, 5.2f, 0.01f, 1.0f / MotorControl::cmPSToPerc, 0.5f / MotorControl::cmPSToPerc, -1.0f / MotorControl::cmPSToPerc, 1.0f / MotorControl::cmPSToPerc);
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
			constexpr JAFD::PIDSettings angularVelPidSettings(0.25f, 1.2f, 0.0f, 10.0f, 5.0f, -10.0f, 10.0f);
		}
	}

	namespace SmoothDriving
	{
		constexpr uint8_t maxArrrayedTasks = 5;		// Maximum number of tasks in TaskArray
		constexpr uint16_t maxAlignDistError = 8;	// Maximum deviation from perfect aligned distance (mm)
		constexpr uint16_t maxAlignStartDist = 50;	// Maximum deviation from aligned distance at beginning to start (mm)
		constexpr uint16_t alignSpeed = MotorControl::minSpeed;		// Minimum speed to align to wall
	}

	namespace Dispenser
	{

	}

	namespace MazeMapping
	{
		constexpr float maxAngleFromStraight = 0.2f;	// Maximum angle (rad) deviation from straight
		constexpr float maxDistFromMiddle = 5.0f;		// Maximum position deviation (x and y) from filed middle
		constexpr uint16_t distLongerThanBorder = 70;	// Distance longer than border from which next field is empty (mm)
	}

	namespace DistanceSensors
	{
		constexpr uint8_t multiplexerAddr = 0x70;

		namespace LeftFront
		{
			constexpr uint8_t multiplexCh = 3;
		}

		namespace LeftBack
		{
			constexpr uint8_t multiplexCh = -1;
		}

		namespace RightFront
		{
			constexpr uint8_t multiplexCh = 2;
		}

		namespace RightBack
		{
			constexpr uint8_t multiplexCh = -1;
		}

		namespace FrontLeft
		{
			constexpr uint8_t multiplexCh = 4;
		}

		namespace FrontRight
		{
			constexpr uint8_t multiplexCh = 5;
		}

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