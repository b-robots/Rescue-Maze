#pragma once

#include "AllDatatypes.h"

namespace SIAL {
	namespace MotorControl {
		// Setup the Motor-Shield
		ReturnCode setup();

		// Get distance measured by encoder
		float getDistance(const Motor motor);

		// Get measured velocity
		WheelSpeeds getSpeeds();

		// Get more accurate measured velocity
		FloatWheelSpeeds getFloatSpeeds();

		void calcMotorSpeed(uint8_t freq);

		// Interrupthandler for speed PID-Loop
		void speedPID(uint8_t freq);

		// Set motor speed (cm/s)
		void setSpeeds(const WheelSpeeds wheelSpeeds);

		// Get the current
		float getCurrent(const Motor motor);
	}
}