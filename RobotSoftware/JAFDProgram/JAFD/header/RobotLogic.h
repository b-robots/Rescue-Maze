/*
This is the heart of the robot
*/

#pragma once

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include "AllDatatypes.h"

namespace JAFD
{
	namespace RobotLogic
	{
		void loop();
		void timeBetweenUpdate();	// Gets executed once while waiting for distance sensor measurements. Has to be faster than ~100ms
	}
}