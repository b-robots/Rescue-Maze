/*
This part of the Library is responsible for driving the motors.
*/

#pragma once

#include "ReturnCode_public.h"
#include "Interrupts_private.h"

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
		ReturnCode motorControlSetup();

		// Get distance measured by encoder
		float getDistance(Motor motor);

		// Interrupthandler for Encoder
		void encoderInterrupt(Interrupts::InterruptSource source, uint32_t isr);

		// Set motor speed
		void setSpeed(Motor motor, float speed);

		// Get the current
		float getCurrent(Motor motor);
	}
}