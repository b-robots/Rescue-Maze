#pragma once

#include "../SIALSettings.h"
#include "../header/AllDatatypes.h"

#include <malloc.h>
#include <stdlib.h>

namespace SIAL
{
	namespace PowerLEDs
	{
		void setup();
		void setBrightness(float perc);
		void setBrightness(bool left, float perc);
	}

	namespace Bumper {
		extern volatile bool left;
		extern volatile bool right;

		void setup();
		void interrupt(InterruptSource source, uint32_t isr);
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