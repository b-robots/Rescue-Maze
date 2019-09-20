/*
This private part of the Library is responsible for dispensing the rescue packages.
*/

#pragma once

#include "implementation/Dispenser_public.h"
#include "implementation/ReturnCode_public.h"

#include <stdint.h>

namespace JAFTD
{
	namespace Dispenser
	{
		// Set up the Dispenser System
		ReturnCode dispenserSetup(DispenserSettings settings);

		// Dispence items
		ReturnCode dispense(uint8_t number); // So könnte es aussehen
	}

}
