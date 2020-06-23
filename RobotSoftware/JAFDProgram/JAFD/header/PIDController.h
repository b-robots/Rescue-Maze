/*
This part is responsible for a PID-Controller
*/

#pragma once

#include <stdint.h>

namespace JAFD
{
	struct PIDSettings
	{
		float kp;			// Proportional gain
		float ki;			// Integral gain
		float kd;			// Differentia gain
		float maxAbsInt;	// Maximum absolute value of integral term to prevent windup
		float maxAbsDiff;	// Macimum absolute value of differential term to prevent massive overshoot
		float minOutput;	// Minimum output
		float maxOutput;	// Maximum output

		constexpr PIDSettings(float kp, float ki, float kd, float maxAbsInt, float maxAbsDiff, float minOutput, float maxOutput) : kp(kp), ki(ki), kd(kd), maxAbsInt(maxAbsInt), maxAbsDiff(maxAbsDiff), minOutput(minOutput), maxOutput(maxOutput) {}
	};

	class PIDController
	{
	private:
		const PIDSettings settings;		// Settings 
		float errorInt;					// Error integral
		float lastErr;					// Last error
		uint32_t lastTimePoint;			// Last time 'process()' has been called in ms.
		bool firstCall;					// Is it the first call to process after a reset?
	public:
		PIDController(const PIDSettings settings);
		float process(const float setPoint, const float currentValue);					// Process inputs and get controller output
		float process(const float setPoint, const float currentValue, const float dt);	// Process inputs and get controller output with specified dt.
		void reset();																	// Reset the controller
	};
}