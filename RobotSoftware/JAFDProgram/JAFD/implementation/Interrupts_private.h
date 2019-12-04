/*
In this part are the InterruptSource enums
*/

#pragma once

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include <stdint.h>

namespace JAFD
{
	namespace Interrupts
	{
		// Interrupt source for PIO Interrupts (equals Periphal ID)
		enum class InterruptSource : uint8_t
		{
			pioA = ID_PIOA,
			pioB = ID_PIOB,
			pioC = ID_PIOC,
			pioD = ID_PIOD
		};
	}
}