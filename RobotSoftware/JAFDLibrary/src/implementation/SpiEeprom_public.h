/*
This public file of the library is responsible for the access to the SPI EEPROM
*/

#pragma once

#include <stdint.h>

namespace JAFD
{
	// Namespace for the SpiEeprom
	namespace SpiEeprom
	{
		// Settings
		typedef struct {
			uint8_t ssPin;
		} SpiEepromSet;
	}
}