/*
This part of the Library is responsible for driving the motors.
*/

#pragma once

namespace JAFD
{
	// Namespace for Dual MC33926 Motor Driver
	namespace MotorControl
	{
		// Settings for Motor-Shield
		typedef struct
		{
			uint8_t mLPWM; // PWM pin left motor
			uint8_t mRPWM; // PWM pin right motor

			uint8_t mLDir; // Direction pin left motor
			uint8_t mRDir; // Direction pin right motor

			uint8_t mLFb; // Current feedback output left motor
			uint8_t mRFb; // Current feedback output right motor
		} MotorControlSettings;
	}
}