
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
#include <Adafruit_TCS34725.h>

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
		constexpr float distSensLeftRightDist = 14.0f;
		constexpr float distSensFrontBackDist = 16.0f;
		constexpr float distSensFrontSpacing = 10.0f;
		constexpr float distSensFrontDistToMiddle = 9.4f;		// !!! ALWAYS = sqrt(distSensFrontBackDist^2 + distSensFrontSpacing^2) / 2
		constexpr float distSensFrontAngleToMiddle = 0.5586f;	// !!! ALWAYS = arctan(distSensFrontSpacing / distSensFrontBackDist)
		constexpr float distSensLRSpacing = 12.5f;
		constexpr float distSensLRDistToMiddle = 9.4f;		// !!! ALWAYS = sqrt(distSensLeftRightDist^2 + distSensLRSpacing^2) / 2
		constexpr float distSensLRAngleToMiddle = 0.729f;	// !!! ALWAYS = arctan(distSensLRSpacing / distSensLeftRightDist)
	}

	namespace SpiNVSRAM
	{
		constexpr uint8_t ssPin = 32;
		constexpr uint32_t mazeMappingStartAddr = 0;
		constexpr uint32_t bno055StartAddr = mazeMappingStartAddr + 64 * 1024;
		constexpr uint32_t distSensStartAddr = bno055StartAddr + 32;
	}

	namespace ColorSensor
	{
		constexpr uint8_t interruptPin = 7;
		constexpr tcs34725IntegrationTime_t tcsIntegrationTime = TCS34725_INTEGRATIONTIME_154MS;
		constexpr tcs34725Gain_t tcsGain = TCS34725_GAIN_1X;
	}

	namespace MotorControl
	{
		constexpr float cmPSToPerc = 1.0f / (97.0f / 60.0f * 3.1415f * Mechanics::wheelDiameter);		// Conversion factor from cm/s to motor PWM duty cycle (NOTE: The conversion isnt linear. This factor is too low for very low speeds and too high for maximum speed.)

		constexpr uint8_t minSpeed = 15;			// Minimum speed for motor to rotate

		constexpr float pulsePerRev = 4741.44f / 4.0f;	// Rotary-Encoder pulses per revolution

		constexpr uint8_t currentADCSampleCount = 2;		// How often to sample and average the ADC measurement for the current
		constexpr float currentSensFactor = 1.0f / 0.14f;	// 140mv/A
		
		constexpr float voltageSensFactor = 0.5f;			// Relation between real voltage and measured voltage

		constexpr float initPWMReduction = 0.82f;			// Starting with this reduction of the pwm to prevent overvoltage
		constexpr float pwmRedIIRFactor = 0.8f;				// IIR factor for PWM reduction value

		namespace Left
		{
			constexpr uint8_t pwmPin = 43;			// PWM pin left motor
			constexpr uint8_t inAPin = 47;			// Input A pin left motor
			constexpr uint8_t inBPin = 48;			// Input B pin left motor
			constexpr uint8_t curFbPin = A11;		// Current feedback output left motor
			constexpr uint8_t voltFbPin = A5;		// Voltage feedback output left motor
			constexpr uint8_t encA = 4;				// Encoder Pin A
			constexpr uint8_t encB = 5;				// Encoder Pin B
		}

		namespace Right
		{
			constexpr uint8_t pwmPin = 41;			// PWM pin left motor
			constexpr uint8_t inAPin = 42;			// Input A pin right motor
			constexpr uint8_t inBPin = 44;			// Input B pin right motor
			constexpr uint8_t curFbPin = A10;		// Current feedback output left motor
			constexpr uint8_t voltFbPin = A6;		// Current feedback output left motor
			constexpr uint8_t encA = 6;				// Encoder Pin A
			constexpr uint8_t encB = 9;				// Encoder Pin B
		}
	}

	namespace SensorFusion
	{
		constexpr float maxPitchForDistSensor = DEG_TO_RAD * 10.0f;		// Maximum pitch of robot for correct front distance measurements
		constexpr uint16_t minDeltaDistForEdge = 30;					// Minimum change in distance that corresponds to an edge (in mm)
		constexpr float distSensSpeedIIRFactor = 0.4f;					// Factor used for IIR-Filter for speed measured by distance sensors
		constexpr float longDistSensIIRFactor = 0.9f;					// Factor used for IIR-Filter for high range distance measurements
		constexpr float shortDistSensIIRFactor = 0.7f;					// Factor used for IIR-Filter for short range distance measurements
		constexpr float minHeightDiffFloor = 50.0f;						// Minimum height differenc to be on the other floor
		constexpr float distSpeedPortion = 0.7f;						// How much is a perfect distance sensor measured speed worth?
		constexpr float bno055Portion = 0.5f;							// How much is a Bno055 measurement worth?
		constexpr float pitchIIRFactor = 0.8f;							// Factor used for IIR-Filter for pitch by BNO055
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
		constexpr uint16_t maxAlignDistError = 10;	// Maximum deviation from perfect aligned distance (mm)
		constexpr uint16_t maxAlignStartDist = 50;	// Maximum deviation from aligned distance at beginning to start (mm)
		constexpr uint16_t alignSpeed = MotorControl::minSpeed;		// Minimum speed to align to wall
		constexpr uint16_t minAlignDist = 70;						// Minimum align distance, is default
	}

	namespace Dispenser
	{
		namespace Left
		{
			constexpr float startDty = 0.03f;	// duty cylce for start
			constexpr float endDty = 0.12f;		// duty cylce for start
			constexpr uint8_t servoPin = 35;
			constexpr uint8_t startCubeCount = 5;
		}

		namespace Right
		{
			constexpr float startDty = 0.12f;	// duty cylce for start
			constexpr float endDty = 0.03f;		// duty cylce for start
			constexpr uint8_t servoPin = 37;
			constexpr uint8_t startCubeCount = 5;
		}
	}

	namespace MazeMapping
	{
		constexpr float distLongerThanBorder = 7.0f;	// Distance longer than border from which next field is empty (cm)
		constexpr float widthSecureDetectFactor = 0.7f;	// Factor of cell width in which border the distance measurement safely hits the front wall	
	}

	namespace DistanceSensors
	{
		constexpr uint16_t minCalibDataDiff = 20;		// Minimum difference in calibration data
		constexpr uint8_t bytesPerCalibData = 8;

		constexpr uint8_t averagingNumSamples = 2;

		constexpr uint8_t multiplexerAddr = 0x70;

		constexpr uint16_t timeout = 150;

		namespace LeftFront
		{
			constexpr uint8_t multiplexCh = 4;
		}

		namespace LeftBack
		{
			constexpr uint8_t multiplexCh = 7;
		}

		namespace RightFront
		{
			constexpr uint8_t multiplexCh = 5;
		}

		namespace RightBack
		{
			constexpr uint8_t multiplexCh = 6;
		}

		namespace FrontLeft
		{
			constexpr uint8_t multiplexCh = 2;
		}

		namespace FrontRight
		{
			constexpr uint8_t multiplexCh = 3;
		}

		namespace FrontLong
		{
			constexpr JAFD::SerialType serialType = JAFD::SerialType::three;
		}
	}
}