/*
This private part of the library is responsible for the AMG8833 heat sensors.
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
	namespace HeatSensor
	{
		ReturnCode setup();
		ReturnCode reset();

		bool detectVictim(HeatSensorSide sensor);
	}
}
