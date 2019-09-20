/*
This public part of the Library is responsible for dispensing the rescue packages.
*/

#pragma once

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

namespace JAFTD
{
	namespace Dispenser
	{
		// Settings for Dispenser
		typedef struct
		{
			int anzahl;
			byte pin1;
		} DispenserSettings;
	}
}