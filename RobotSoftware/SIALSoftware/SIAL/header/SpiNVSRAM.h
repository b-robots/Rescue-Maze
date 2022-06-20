#pragma once

#include <SPI.h>

#include "AllDatatypes.h"

namespace SIAL
{
	// Namespace for the SPI NVSRAM "23LCV1024"
	namespace SpiNVSRAM
	{
		// Helping functions
		void enable();
		void disable();

		// Devise specific constants
		constexpr uint16_t pageSize = 256;

		// Init
		ReturnCode setup();

		// Read and write functions
		uint8_t readByte(const uint32_t address);
		void writeByte(const uint32_t address, const uint8_t byte);
		void readStream(const uint32_t address, uint8_t* buffer, const uint32_t length);
		void writeStream(uint32_t address, uint8_t* buffer, const uint32_t length);
	}
}