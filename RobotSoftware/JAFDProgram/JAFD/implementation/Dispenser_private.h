/*
This private part of the Library is responsible for dispensing the rescue packages.
*/

#pragma once

#include "ReturnCode_public.h"

#include <stdint.h>

namespace JAFD
{
	namespace Dispenser
	{
		// Set up the Dispenser System
		ReturnCode dispenserSetup();

		// Dispence items
		ReturnCode dispense(uint8_t number); // So könnte es aussehen
	}

}
