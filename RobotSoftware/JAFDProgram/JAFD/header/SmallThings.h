#pragma once

#include "../../JAFDSettings.h"
#include "../header/AllDatatypes.h"

#include <malloc.h>
#include <stdlib.h>

namespace JAFD
{
	namespace PowerLEDs
	{
		ReturnCode setup();
		void setBrightness(float perc);
	}

	namespace I2CBus
	{
		ReturnCode setup();
		ReturnCode resetBus();
	}

	namespace MemWatcher
	{
		uint32_t getDynamicRam();
		uint32_t getStackRam();
		uint32_t getFreeRam();
	}

	namespace Wait
	{
		void delayUnblocking(uint32_t ms);

		void waitForFinishedTask();
	}

	// TODO
	namespace Switch
	{
		void setup();
		bool getState();
	}
	
	// TODO
	namespace Buzzer
	{

	}
}