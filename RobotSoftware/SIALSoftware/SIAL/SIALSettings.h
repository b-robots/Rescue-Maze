#pragma once

#include "header/AllDatatypes.h"
#include "header/PIDController.h"

namespace SIALSettings {
	namespace Mechanics
	{
		constexpr float wheelDiameter = 8.2f;
		constexpr float wheelDistance = 15.5f;
		constexpr float axialSpacing = 9.7f;
		constexpr float wheelDistToMiddle = 8.78f;			// !!! ALWAYS = sqrt(axialSpacing^2 + wheelDistance^2) / 2
		constexpr float distSensLeftRightDist = 12.2f;
		constexpr float distSensFrontBackDist = 15.0f;
		constexpr float distSensFrontSpacing = 9.2f;
		constexpr float distSensFrontDistToMiddle = 8.8f;		// !!! ALWAYS = sqrt(distSensFrontBackDist^2 + distSensFrontSpacing^2) / 2
		constexpr float distSensFrontAngleToMiddle = 0.55f;	// !!! ALWAYS = arctan(distSensFrontSpacing / distSensFrontBackDist)
		constexpr float distSensLRSpacing = 10.0f;
		constexpr float distSensLRDistToMiddle = 7.9f;		// !!! ALWAYS = sqrt(distSensLeftRightDist^2 + distSensLRSpacing^2) / 2
		constexpr float distSensLRAngleToMiddle = 0.687f;	// !!! ALWAYS = arctan(distSensLRSpacing / distSensLeftRightDist)
		constexpr float robotLength = 20.0f;
		constexpr float robotWidth = 14.0f;
	}

	namespace MotorControl
	{
		constexpr float cmPSToPerc = 1.0f / (97.0f / 60.0f * M_PI * Mechanics::wheelDiameter);		// Conversion factor from cm/s to motor PWM duty cycle (NOTE: The conversion isnt linear. This factor is too low for very low speeds and too high for maximum speed.)

		constexpr uint8_t minSpeed = 5;						// Minimum speed for motor to rotate
		constexpr uint8_t maxSpeed = 1.0f / cmPSToPerc;		// Calculated maximum speed

		constexpr float pulsePerRev = 4741.44f;	// Rotary-Encoder pulses per revolution

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
			constexpr bool swapTCInputs = true;
			constexpr auto tc = TC2;
		}

		namespace Right
		{
			constexpr uint8_t pwmPin = 41;			// PWM pin left motor
			constexpr uint8_t inAPin = 48;			// Input A pin right motor
			constexpr uint8_t inBPin = 46;			// Input B pin right motor
			constexpr uint8_t curFbPin = A9;		// Current feedback output left motor
			constexpr uint8_t voltFbPinA = A7;		// Voltage feedback output left motor / A
			constexpr uint8_t voltFbPinB = A5;		// Voltage feedback output left motor / b
			constexpr uint8_t encA = 2;				// Encoder Pin A
			constexpr uint8_t encB = 13;			// Encoder Pin B
			constexpr bool swapTCInputs = true;
			constexpr auto tc = TC0;
		}
	}

	namespace Controller
	{
		namespace Motor
		{
			constexpr SIAL::PIDSettings pidSettings(0.67f, 9.5f, 0.03f, 1.0f / MotorControl::cmPSToPerc, 0.5f / MotorControl::cmPSToPerc, -1.0f / MotorControl::cmPSToPerc, 1.0f / MotorControl::cmPSToPerc);
		}

		namespace GoToAngle
		{
			constexpr SIAL::PIDSettings pidSettings(0.085f, 0.005f, 0.01f, 3.0f, 3.0f, -3.0f, 3.0f);
			constexpr float aheadDistL = 20.0f;
		}
	}

	namespace SensorFiltering {
		namespace Encoder {
			constexpr float IIRFac = 0.9f;
		}
	}

	namespace SensorFusion
	{
		// Distance & Speed
		constexpr float longDistSensIIRFactor = 0.8f;					// Factor used for IIR-Filter for high range distance measurements
		constexpr float shortDistSensIIRFactor = 0.9f;					// Factor used for IIR-Filter for short range distance measurements
		constexpr float speedIIRFac = 1.0f;

		// Rotation
		constexpr float chi = 1.2f;
		constexpr float bno055DiffPortion = 1.0f;						// How much is the differentiation of the BNO055 angle worth?
		constexpr float bno055RotPortion = 1.0f;						// How much is a Bno055 rotation measurement worth?
		constexpr float angularVelIIRFactor = 0.9f;						// Factor used for IIR-Filter for angular velocity
		constexpr float pitchIIRFactor = 0.8f;							// Factor used for IIR-Filter for pitch angle
		constexpr float distAngularPortion = 0.3f;						// How much is a perfect distance sensor measured angle worth?
		constexpr float angleDiffPortion = 0.0f;						// How much is the heading influenced by the angular velocity instead of the absolute orientation values?
	}

	namespace Bumper {
		constexpr uint8_t leftPin = 15;
		constexpr uint8_t rightPin = 14;
	}

	namespace MazeMapping
	{
		constexpr float minRampAngle = 0.18f;
		constexpr uint8_t maxDistLongerThanBorder = 5;
	}

	namespace Dispenser
	{
		constexpr uint32_t pause = 700;		// How long is the piston extended in ms?

		namespace Left
		{
			constexpr float startDty = 0.11f;		// duty cylce for start
			constexpr float endDty = 0.077f;		// duty cylce for end
			constexpr uint8_t servoPin = 35;
			constexpr uint8_t startCubeCount = 6;
		}

		namespace Right
		{
			constexpr float startDty = 0.035f;		// duty cylce for start
			constexpr float endDty = 0.067f;			// duty cylce for end
			constexpr uint8_t servoPin = 37;
			constexpr uint8_t startCubeCount = 6;
		}
	}

	namespace HeatSensors
	{
		constexpr float threshold = 7.0f;

		namespace Left
		{
			constexpr uint8_t i2cChannel = 7;
		}

		namespace Right
		{
			constexpr uint8_t i2cChannel = 6;
		}
	}

	namespace DistanceSensors
	{
		constexpr uint16_t minCalibDataDiff = 20;		// Minimum difference in calibration data
		constexpr uint8_t bytesPerCalibData = 8;

		constexpr uint8_t multiplexerAddr = 0x70;

		constexpr uint16_t timeout = 80;

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
			constexpr uint8_t multiplexCh = 1;
		}

		namespace FrontRight
		{
			constexpr uint8_t multiplexCh = 0;
		}

		namespace FrontLong
		{
			constexpr SIAL::SerialType serialType = SIAL::SerialType::two;
		}
	}


	namespace SpiNVSRAM
	{
		constexpr uint8_t ssPin = 39;
		constexpr uint32_t mazeMappingStartAddr = 0;
		constexpr uint32_t bno055StartAddr = mazeMappingStartAddr + 64 * 1024;
		constexpr uint32_t distSensStartAddr = bno055StartAddr + 32;
	}

	namespace PowerLEDs
	{
		constexpr float blinkPeriod = 500.0f;

		constexpr float defaultPower = 0.5f;

		namespace Left
		{
			constexpr uint8_t pwmPin = 8;
		}

		namespace Right
		{
			constexpr uint8_t pwmPin = 7;
		}
	}

	namespace Switch
	{
		constexpr uint8_t pin = 12;
	}
}