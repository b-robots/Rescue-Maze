/*
This file of the library is responsible for the sensor fusion
*/

#pragma once

#include "../header/SensorFusion.h"
#include "../header/MotorControl.h"
#include "../../JAFDSettings.h"

namespace JAFD
{
	namespace SensorFusion
	{
		namespace
		{
			volatile RobotState _robotState; // Current state of robot
		}

		void updateSensorValues(const uint8_t freq)
		{
			_robotState.wheelSpeeds = MotorControl::getFloatSpeeds();
			_robotState.angularVel = Vec3f((_robotState.wheelSpeeds.right - _robotState.wheelSpeeds.left) / JAFDSettings::Mechanics::wheelDistance, 0.0f, 0.0f);
			_robotState.rotation += _robotState.angularVel / (float)freq;
			_robotState.forwardVel = (_robotState.wheelSpeeds.left + _robotState.wheelSpeeds.right) / 2.0f;
			_robotState.position += Vec3f(cosf(_robotState.rotation.x), sinf(_robotState.rotation.x), 0.0f) * (_robotState.forwardVel / (float)freq) + Vec3f(0.0f, 3.0f / (float)(freq), 0.0f);
		}

		RobotState getRobotState()
		{
			return _robotState;
		}
	}
}