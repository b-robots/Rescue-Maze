/*
This private part of the Library is responsible for dispensing the rescue packages.
*/

#pragma once

#include "RobotLibraryIncludes.h"
#include "Helper_private.h"
#include "Dispenser_public.h"

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
