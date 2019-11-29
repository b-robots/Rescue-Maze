/*
This private file of the library is responsible for the access to the SPI NVSRAM
*/

#pragma once

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include <stdint.h>

#include "ReturnCode_public.h"

namespace JAFD
{
	namespace SensorFusion
	{
	}
}