/*
This part of the Library is responsible for driving the motors.
*/

#pragma once

#include "MotorShield_public.h"
#include "ReturnCode_public.h"

namespace JAFD
{
	namespace MotorShield
	{
		// Setup the Motor-Shield
		ReturnCode motorShieldSetup(MotorShieldSettings settings);
	}
}