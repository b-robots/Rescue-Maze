/*
This part is responsible for dispensing the rescue packages.
*/
#pragma once

#include "AllDatatypes.h"

namespace JAFD
{
	namespace Dispenser
	{
		// Set up the Dispenser System
		ReturnCode setup();

		// Dispence items
		ReturnCode dispense(uint8_t number); // So könnte es aussehen
	}

}
