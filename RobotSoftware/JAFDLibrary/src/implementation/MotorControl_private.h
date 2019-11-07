/*
This part of the Library is responsible for driving the motors.
*/

#pragma once

#include "MotorControl_public.h"
#include "ReturnCode_public.h"

namespace JAFD
{
	namespace MotorControl
	{
		enum class Motor : uint8_t
		{
			left,
			right
		};

		// Setup the Motor-Shield
		ReturnCode motorControlSetup(MotorControlSettings settings);

		// Set motor speed
		void setSpeed(Motor motor, float speed);

		// Get the current
		float getCurrent(Motor motor);
	}
}