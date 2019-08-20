/*
This part of the Library is responsible for dispensing the rescue packages.
*/

#pragma once

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include <Servo.h>
#include "Helper.h"

namespace JAFTD
{
	namespace internal
	{
		namespace Dispenser
		{
			// Set up the Dispenser System
			ReturnCode dispenserSetup();

			// Dispence items
			ReturnCode dispense(uint8_t number); // So könnte es aussehen
		}
	}
}
