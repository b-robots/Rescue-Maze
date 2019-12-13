/*
This file of the library is responsible for the sensor fusion
*/

#pragma once

#include "../header/AllDatatypes.h"
#include "../header/SensorFusion.h"
#include "../header/MotorControl.h"

namespace JAFD
{
	namespace SensorFusion
	{
		void updateSensorValues(const uint8_t freq)
		{
			robotState.wheelSpeeds = MotorControl::getSpeeds();
			robotState.position += Vec2f((robotState.wheelSpeeds.left + robotState.wheelSpeeds.left) / (float)freq / 2.0f, 0.0f);
		}
	}
}