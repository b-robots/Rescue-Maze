#pragma once

#include "AllDatatypes.h"

namespace SIAL
{
	namespace I2CMultiplexer
	{
		constexpr uint8_t maxCh = 7;
		ReturnCode setup();
		uint8_t getChannel();
		uint8_t selectChannel(uint8_t channel);
		bool checkI2C();
		void recoverI2C();
	}
}