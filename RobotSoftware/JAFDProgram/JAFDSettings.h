
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

// We use the TPA81 at the moment
//#define USE_AMG8833

namespace JAFDSettings
{
	namespace Switch
	{
		constexpr uint8_t pin = 12;
	}

	namespace Field
	{
		constexpr float cellWidth = 30.0f;	// Cell width in cm
		constexpr float minObstacleSpace = 20.0f;	// Minimum available space between obstacle and border
	}

	namespace Mechanics
	{
		constexpr float wheelDiameter = 8.0f;
		constexpr float wheelDistance = 15.0f;
		constexpr float axialSpacing = 9.7f;
		constexpr float wheelDistToMiddle = 8.93f;			// !!! ALWAYS = sqrt(axialSpacing^2 + wheelDistance^2) / 2
		constexpr float distSensLeftRightDist = 12.2f;
		constexpr float distSensFrontBackDist = 13.0f;
		constexpr float distSensFrontSpacing = 10.5f;
		constexpr float distSensFrontDistToMiddle = 8.4f;		// !!! ALWAYS = sqrt(distSensFrontBackDist^2 + distSensFrontSpacing^2) / 2
		constexpr float distSensFrontAngleToMiddle = 0.6794f;	// !!! ALWAYS = arctan(distSensFrontSpacing / distSensFrontBackDist)
		constexpr float distSensLRSpacing = 11.0f;
		constexpr float distSensLRDistToMiddle = 8.2f;		// !!! ALWAYS = sqrt(distSensLeftRightDist^2 + distSensLRSpacing^2) / 2
		constexpr float distSensLRAngleToMiddle = 0.7337f;	// !!! ALWAYS = arctan(distSensLRSpacing / distSensLeftRightDist)
		constexpr float robotLength = 20.0f;
		constexpr float robotWidth = 14.0f;
	}

	namespace SpiNVSRAM
	{
		constexpr uint8_t ssPin = 39;
		constexpr uint32_t mazeMappingStartAddr = 0;
		constexpr uint32_t bno055StartAddr = mazeMappingStartAddr + 64 * 1024;
		constexpr uint32_t distSensStartAddr = bno055StartAddr + 32;
	}

	namespace ColorSensor
	{
		constexpr uint8_t interruptPin = 34;
		constexpr tcs34725IntegrationTime_t tcsIntegrationTime = TCS34725_INTEGRATIONTIME_154MS;
		constexpr tcs34725Gain_t tcsGain = TCS34725_GAIN_1X;
	}

	namespace CamRec
	{
	}

	namespace MotorControl
	{
		constexpr float cmPSToPerc = 1.0f / (97.0f / 60.0f * M_PI * Mechanics::wheelDiameter);		// Conversion factor from cm/s to motor PWM duty cycle (NOTE: The conversion isnt linear. This factor is too low for very low speeds and too high for maximum speed.)

		constexpr uint8_t minSpeed = 10;					// Minimum speed for motor to rotate
		constexpr uint8_t maxSpeed = 1.0f / cmPSToPerc;		// Calculated maximum speed
		constexpr float maxRotSpeed = 2.0f * maxSpeed / Mechanics::wheelDistance;	// Calculated maximum rotation speed

		constexpr float pulsePerRev = 4741.44f / 4.0f;	// Rotary-Encoder pulses per revolution

		constexpr uint8_t currentADCSampleCount = 2;		// How often to sample and average the ADC measurement for the current
		constexpr float currentSensFactor = 1.0f / 0.14f;	// 140mv/A
		
		constexpr float voltageSensFactor = 2.585f;			// "Real Voltage" / "Measured Voltage" for voltage feedback

		constexpr float initPWMReduction = 0.71f;			// Starting with this reduction of the pwm to prevent overvoltage
		constexpr float pwmRedIIRFactor = 0.5f;				// IIR factor for PWM reduction value

		namespace Left
		{
			constexpr uint8_t pwmPin = 43;			// PWM pin left motor
			constexpr uint8_t inAPin = 49;			// Input A pin left motor
			constexpr uint8_t inBPin = 47;			// Input B pin left motor
			constexpr uint8_t curFbPin = A8;		// Current feedback output left motor
			constexpr uint8_t voltFbPinA = A1;		// Voltage feedback output left motor / A
			constexpr uint8_t voltFbPinB = A3;		// Voltage feedback output left motor / B
			constexpr uint8_t encA = 4;				// Encoder Pin A
			constexpr uint8_t encB = 5;				// Encoder Pin B
		}

		namespace Right
		{
			constexpr uint8_t pwmPin = 41;			// PWM pin left motor
			constexpr uint8_t inAPin = 48;			// Input A pin right motor
			constexpr uint8_t inBPin = 46;			// Input B pin right motor
			constexpr uint8_t curFbPin = A9;		// Current feedback output left motor
			constexpr uint8_t voltFbPinA = A7;		// Voltage feedback output left motor / A
			constexpr uint8_t voltFbPinB = A5;		// Voltage feedback output left motor / b
			constexpr uint8_t encA = 6;				// Encoder Pin A
			constexpr uint8_t encB = 9;				// Encoder Pin B
		}
	}

	namespace SensorFusion
	{
		// Maze
		constexpr float maxPitchForDistSensor = DEG_TO_RAD * 10.0f;		// Maximum pitch of robot for correct front distance measurements
		constexpr uint16_t minDeltaDistForEdge = 30;					// Minimum change in distance that corresponds to an edge (in mm)

		// Distance & Speed
		constexpr float distSensSpeedIIRFactor = 0.8f;					// Factor used for IIR-Filter for speed measured by distance sensors
		constexpr float longDistSensIIRFactor = 0.8f;					// Factor used for IIR-Filter for high range distance measurements
		constexpr float shortDistSensIIRFactor = 0.8f;					// Factor used for IIR-Filter for short range distance measurements
		constexpr float distSpeedPortion = 0.2f;						// How much is a perfect distance sensor measured speed worth?

		// Position
		constexpr float distSensOffsetPortion = 1.0f;					// How much does the center-offset measured by all distance sensors count?

		// Rotation
		constexpr float bno055RotPortion = 0.1f;						// How much is a Bno055 rotation measurement worth?
		constexpr float angularVelIIRFactor = 0.9f;						// Factor used for IIR-Filter for angular velocity
		constexpr float angularVelDiffPortion = 0.5f;					// How much of the angular yaw velocity is based on differentiation?
		constexpr float pitchIIRFactor = 0.5f;							// Factor used for IIR-Filter for pitch angle
		constexpr float distAngularPortion = 1.0f;						// How much is a perfect distance sensor measured angle worth?
		constexpr float maxAngleDeviation = 90.0f * DEG_TO_RAD;			// If there is a heading deviation greater than this after one time step (50ms) the BNO had an error
	}

	namespace Controller
	{
		namespace Motor
		{
			constexpr JAFD::PIDSettings pidSettings(0.85f, 5.2f, 0.01f, 1.0f / MotorControl::cmPSToPerc, 0.5f / MotorControl::cmPSToPerc, -1.0f / MotorControl::cmPSToPerc, 1.0f / MotorControl::cmPSToPerc);
		}

		namespace GoToAngle
		{
			constexpr float turningGainConstant = 0.1f;
			constexpr float aheadDistL = Mechanics::wheelDistance / (2.0f * turningGainConstant);
			constexpr float angleDampingBegin = 10.0f;
		}

		namespace PID
		{
			constexpr float nonePIDPart = 0.5f;
		}

		namespace SmoothDriving
		{
			constexpr JAFD::PIDSettings forwardVelPidSettings(0.3f, 1.5f, 0.01f, 1.0f / MotorControl::cmPSToPerc, 0.5f / MotorControl::cmPSToPerc, -1.0f / MotorControl::cmPSToPerc, 1.0f / MotorControl::cmPSToPerc);
			constexpr JAFD::PIDSettings angularVelPidSettings(0.25f, 2.0f, 0.15f, 10.0f, 5.0f, -10.0f, 10.0f);
		}
	}

	namespace SmoothDriving
	{
		constexpr uint8_t maxArrrayedTasks = 5;						// Maximum number of tasks in TaskArray
		constexpr uint16_t maxAlignDistError = 10;					// Maximum deviation from perfect aligned distance (mm)
		constexpr uint16_t maxAlignStartDist = 50;					// Maximum deviation from aligned distance at beginning to start (mm)
		constexpr uint16_t alignSpeed = MotorControl::minSpeed;		// Minimum speed to align to wall
		constexpr uint16_t minAlignDist = 70;						// Minimum align distance, is default
	}

	namespace Dispenser
	{
		constexpr uint32_t pause = 700;		// How long is the piston extended in ms?

		namespace Left
		{
			constexpr float startDty = 0.03f;	// duty cylce for start
			constexpr float endDty = 0.12f;		// duty cylce for start
			constexpr uint8_t servoPin = 35;
			constexpr uint8_t startCubeCount = 6;
		}

		namespace Right
		{
			constexpr float startDty = 0.12f;	// duty cylce for start
			constexpr float endDty = 0.03f;		// duty cylce for start
			constexpr uint8_t servoPin = 37;
			constexpr uint8_t startCubeCount = 6;
		}
	}

	namespace MazeMapping
	{
		constexpr float distLongerThanBorder = 7.0f;		// Distance longer than border from which next field is empty (cm)
		constexpr float widthSecureDetectFactor = 0.85f;	// Factor of cell width in which border the distance measurement safely hits the front wall	
	}

	namespace DistanceSensors
	{
		constexpr uint16_t minCalibDataDiff = 20;		// Minimum difference in calibration data
		constexpr uint8_t bytesPerCalibData = 8;

		constexpr uint8_t multiplexerAddr = 0x70;

		constexpr uint16_t timeout = 200;

		namespace LeftFront
		{
			constexpr uint8_t multiplexCh = 2;
		}

		namespace LeftBack
		{
			constexpr uint8_t multiplexCh = 3;
		}

		namespace RightFront
		{
			constexpr uint8_t multiplexCh = 4;
		}

		namespace RightBack
		{
			constexpr uint8_t multiplexCh = 5;
		}

		namespace FrontLeft
		{
			constexpr uint8_t multiplexCh = 0;
		}

		namespace FrontRight
		{
			constexpr uint8_t multiplexCh = 1;
		}

		namespace FrontLong
		{
			constexpr JAFD::SerialType serialType = JAFD::SerialType::two;
		}
	}

	namespace I2CBus
	{
		constexpr uint8_t powerResetPin = 38;
	}

	namespace PowerLEDs
	{
		constexpr float defaultPower = 0.4f;

		namespace Left
		{
			constexpr uint8_t pwmPin = 8;
		}

		namespace Right
		{
			constexpr uint8_t pwmPin = 7;
		}
	}

	namespace HeatSensors
	{
		constexpr uint8_t averagingNumber = 5;

		constexpr float threshold = 10.0f;

		namespace Left
		{
#ifndef USE_AMG8833
			constexpr uint8_t i2cChannel = 7;
#else
			constexpr uint8_t i2cAddr = 0x69;
#endif
		}

		namespace Right
		{
#ifndef USE_AMG8833
			constexpr uint8_t i2cChannel = 6;
#else
			constexpr uint8_t i2cAddr = 0x68;
#endif
		}
	}
}