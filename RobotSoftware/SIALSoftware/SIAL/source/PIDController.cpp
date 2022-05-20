#include <Arduino.h>
#include "../header/PIDController.h"

namespace SIAL
{
	PIDController::PIDController(const PIDSettings settings) : settings(settings), errorInt(0.0f), lastErr(0.0f), lastTimePoint(0.0f), firstCall(true) {}

	float PIDController::process(const float setPoint, const float currentValue)
	{
		const uint32_t currentTime = millis();
		const float dt = (currentTime - lastTimePoint) / 1000.0f;
		lastTimePoint = currentTime;

		const float error = setPoint - currentValue;
		float diffTerm = (error - lastErr) / dt;

		if (firstCall) diffTerm = 0.0f;
		else if (diffTerm > settings.maxAbsDiff) diffTerm = settings.maxAbsDiff;
		else if (diffTerm < -settings.maxAbsDiff) diffTerm = -settings.maxAbsDiff;

		errorInt += lastErr * dt;

		if (firstCall) errorInt = 0.0f;
		else if (errorInt > settings.maxAbsInt) errorInt = settings.maxAbsInt;
		else if (errorInt < -settings.maxAbsInt) errorInt = -settings.maxAbsInt;

		lastErr = error;

		if (firstCall) firstCall = false;

		float output = settings.kp * error + settings.ki * errorInt + settings.kd * diffTerm;

		if (output > settings.maxOutput) output = settings.maxOutput;
		else if (output < settings.minOutput) output = settings.minOutput;

		return output;
	}

	float PIDController::process(const float setPoint, const float currentValue, const float dt)
	{
		const float error = setPoint - currentValue;
		float diffTerm = (error - lastErr) / dt;

		if (firstCall) diffTerm = 0.0f;
		else if (diffTerm > settings.maxAbsDiff) diffTerm = settings.maxAbsDiff;
		else if (diffTerm < -settings.maxAbsDiff) diffTerm = -settings.maxAbsDiff;

		errorInt += lastErr * dt;

		if (firstCall) errorInt = 0.0f;
		else if (errorInt > settings.maxAbsInt) errorInt = settings.maxAbsInt;
		else if (errorInt < -settings.maxAbsInt) errorInt = -settings.maxAbsInt;

		lastErr = error;

		if (firstCall) firstCall = false;

		float output = settings.kp * error + settings.ki * errorInt + settings.kd * diffTerm;

		if (output > settings.maxOutput) output = settings.maxOutput;
		else if (output < settings.minOutput) output = settings.minOutput;

		return output;
	}

	void PIDController::reset()
	{
		errorInt = 0.0f;
		lastErr = 0.0f;
		lastTimePoint = 0;
		firstCall = true;
	}
}