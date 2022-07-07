#pragma once

#include "AllDatatypes.h"

namespace SIAL
{
	namespace Dispenser
	{

		// Set up the Dispenser System
		ReturnCode setup();

		// Dispence items
		ReturnCode dispenseLeft(uint8_t number);
		ReturnCode dispenseRight(uint8_t number);
	}
}