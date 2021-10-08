#pragma once

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include "AllDatatypes.h"

namespace JAFD
{
	namespace I2CMultiplexer
	{
		constexpr uint8_t maxCh = 8;
		ReturnCode setup();
		uint8_t getChannel();
		uint8_t selectChannel(uint8_t channel);
	}
}