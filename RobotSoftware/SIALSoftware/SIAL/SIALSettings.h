#pragma once

#include "header/AllDatatypes.h"
#include "header/PIDController.h"

namespace SIALSettings {
	namespace Mechanics
	{
		constexpr float wheelDiameter = 8.0f;
		constexpr float wheelDistance = 15.0f;
		constexpr float axialSpacing = 9.7f;
		constexpr float wheelDistToMiddle = 8.93f;			// !!! ALWAYS = sqrt(axialSpacing^2 + wheelDistance^2) / 2
		constexpr float distSensLeftRightDist = 12.2f;
		constexpr float distSensFrontBackDist = 14.0f;
		constexpr float distSensFrontSpacing = 9.2f;
		constexpr float distSensFrontDistToMiddle = 8.4f;		// !!! ALWAYS = sqrt(distSensFrontBackDist^2 + distSensFrontSpacing^2) / 2
		constexpr float distSensFrontAngleToMiddle = 0.581f;	// !!! ALWAYS = arctan(distSensFrontSpacing / distSensFrontBackDist)
		constexpr float distSensLRSpacing = 10.0f;
		constexpr float distSensLRDistToMiddle = 7.9f;		// !!! ALWAYS = sqrt(distSensLeftRightDist^2 + distSensLRSpacing^2) / 2
		constexpr float distSensLRAngleToMiddle = 0.687f;	// !!! ALWAYS = arctan(distSensLRSpacing / distSensLeftRightDist)
		constexpr float robotLength = 20.0f;
		constexpr float robotWidth = 14.0f;
	}

	namespace MotorControl
	{
		constexpr float cmPSToPerc = 1.0f / (97.0f / 60.0f * M_PI * Mechanics::wheelDiameter);		// Conversion factor from cm/s to motor PWM duty cycle (NOTE: The conversion isnt linear. This factor is too low for very low speeds and too high for maximum speed.)

		constexpr uint8_t minSpeed = 5;					// Minimum speed for motor to rotate
		constexpr uint8_t maxSpeed = 1.0f / cmPSToPerc;		// Calculated maximum speed
		constexpr float maxRotSpeed = 2.0f * maxSpeed / Mechanics::wheelDistance;	// Calculated maximum rotation speed

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
			constexpr SIAL::PIDSettings pidSettings(0.7f, 4.0f, 0.0f, 1.0f / MotorControl::cmPSToPerc, 0.5f / MotorControl::cmPSToPerc, -1.0f / MotorControl::cmPSToPerc, 1.0f / MotorControl::cmPSToPerc);
		}
	}

	namespace SensorFiltering {
		namespace Encoder {
			constexpr float IIRFac = 0.95f;
		}
	}
}