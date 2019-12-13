/*
This part of the Library is responsible for driving the motors.
*/

#pragma once

#include "AllDatatypes.h"
#include "Interrupts.h"

#include <stdint.h>

namespace JAFD
{
	namespace MotorControl
	{
		// Setup the Motor-Shield
		ReturnCode motorControlSetup();

		// Get distance measured by encoder
		float getDistance(const Motor motor);

		// Get measured velocity
		float getSpeed(const Motor motor);

		// Interrupthandler for speed calculation
		void calcMotorSpeed(const uint8_t dt);

		// Interrupthandler for speed PID-Loop
		void speedPID(const uint8_t dt);

		// Interrupthandler for Encoder
		void encoderInterrupt(const Interrupts::InterruptSource source, const uint32_t isr);

		// Set motor speed (cm/2)
		void setSpeed(const Motor motor, uint8_t speed);

		// Get the current
		float getCurrent(const Motor motor);
	}
}