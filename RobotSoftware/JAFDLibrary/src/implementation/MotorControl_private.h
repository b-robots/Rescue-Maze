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
		// Setup the Motor-Shield
		ReturnCode motorControlSetup(MotorControlSettings settings);

		// Set motor speed / just pfusch
		void setSpeed(uint8_t motor, float speed);
	}
}