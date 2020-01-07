/*
This part is responsible for a PID-Controller
*/

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include "../header/PIDController.h"

namespace JAFD
{
	PIDController::PIDController(const PIDSettings settings) : _settings(settings), _errorInt(0.0f), _lastErr(0.0f), _lastTimePoint(0.0f), _firstCall(true) {}
	
	float PIDController::process(const float setPoint, const float currentValue)
	{
		const uint32_t currentTime = millis();
		const float dt = (currentTime - _lastTimePoint) / 1000.0f;
		_lastTimePoint = currentTime;

		const float error = setPoint - currentValue;
		float diffTerm = (error - _lastErr) / dt;

		if (_firstCall) diffTerm = 0.0f;
		else if (diffTerm > _settings.maxAbsDiff) diffTerm = _settings.maxAbsDiff;
		else if (diffTerm < -_settings.maxAbsDiff) diffTerm = -_settings.maxAbsDiff;

		_errorInt += _lastErr * dt;

		if (_firstCall) _errorInt = 0.0f;
		else if (_errorInt > _settings.maxAbsInt) _errorInt = _settings.maxAbsInt;
		else if (_errorInt < -_settings.maxAbsInt) _errorInt = -_settings.maxAbsInt;

		_lastErr = error;

		if (_firstCall) _firstCall = false;

		float output = _settings.kp * error + _settings.ki * _errorInt + _settings.kd * diffTerm;

		if (output > _settings.maxOutput) output = _settings.maxOutput;
		else if (output < _settings.minOutput) output = _settings.minOutput;

		return output;
	}

	float PIDController::process(const float setPoint, const float currentValue, const float dt)
	{
		const float error = setPoint - currentValue;
		float diffTerm = (error - _lastErr) / dt;

		if (_firstCall) diffTerm = 0.0f;
		else if (diffTerm > _settings.maxAbsDiff) diffTerm = _settings.maxAbsDiff;
		else if (diffTerm < -_settings.maxAbsDiff) diffTerm = -_settings.maxAbsDiff;

		_errorInt += _lastErr * dt;

		if (_firstCall) _errorInt = 0.0f;
		else if (_errorInt > _settings.maxAbsInt) _errorInt = _settings.maxAbsInt;
		else if (_errorInt < -_settings.maxAbsInt) _errorInt = -_settings.maxAbsInt;

		_lastErr = error;

		if (_firstCall) _firstCall = false;

		float output = _settings.kp * error + _settings.ki * _errorInt + _settings.kd * diffTerm;

		if (output > _settings.maxOutput) output = _settings.maxOutput;
		else if (output < _settings.minOutput) output = _settings.minOutput;

		return output;
	}

	void PIDController::reset()
	{
		_errorInt = 0.0f;
		_lastErr = 0.0f;
		_lastTimePoint = 0;
		_firstCall = true;
	}
}