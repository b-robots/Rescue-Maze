#pragma once

#include "../SIALSettings.h"
#include "../header/AllDatatypes.h"

#include <malloc.h>
#include <stdlib.h>

namespace SIAL
{
	namespace PowerLEDs
	{
		ReturnCode setup();
		void setBrightness(float perc);
		void setBrightness(bool left, float perc);
	}

	namespace MemWatcher
	{
		uint32_t getDynamicRam();
		uint32_t getStackRam();
		uint32_t getFreeRam();
	}

	namespace Switch
	{
		void setup();
		bool getState();
	}
}