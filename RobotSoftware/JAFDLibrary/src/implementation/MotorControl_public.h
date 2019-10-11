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
			uint8_t m1PWM; // PWM pin motor 1
			uint8_t m2PWM; // PWM pin motor 2

			uint8_t m1Dir; // Direction pin motor 1
			uint8_t m2Dir; // Direction pin motor 2

			uint8_t m1Fb; // Current sense output motor 1
			uint8_t m2Fb; // Current sense output motor 2
		} MotorControlSettings;
	}
}