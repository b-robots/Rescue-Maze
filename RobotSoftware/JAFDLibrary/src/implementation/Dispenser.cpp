/*
This part of the Library is responsible for dispensing the rescue packages.
*/

#include "Dispenser_private.h"

namespace JAFD
{
	namespace Dispenser
	{
		// Set up the Dispenser System
		ReturnCode dispenserSetup(DispenserSettings settings)
		{
			// Do something...

			return ReturnCode::ok;
		}

		// Dispence items
		ReturnCode dispense(uint8_t number)
		{
			// Auswerfen...

			if (/*irgendwas geht schief*/0)
			{
				return ReturnCode::error;
			}

			return ReturnCode::ok;
		}
	}
}