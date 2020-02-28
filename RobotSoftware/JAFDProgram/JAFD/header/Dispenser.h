/*
This part is responsible for dispensing the rescue packages.
*/
#pragma once

#include "AllDatatypes.h"
#include "DistanceSensors.h"

namespace JAFD
{
	namespace Dispenser
	{

		// Set up the Dispenser System
		ReturnCode setup();

		// Dispence items
		ReturnCode dispenseLeft(uint8_t number); 
		ReturnCode dispenseRight(uint8_t number);

		uint16_t getLeftCubeCount();
		uint16_t getRightCubeCount();
	}

}
