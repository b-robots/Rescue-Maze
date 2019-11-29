/*
This private file of the library is responsible for the access to the SPI NVSRAM
*/

#pragma once

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include <SPI.h>
#include <stdint.h>

#include "ReturnCode_public.h"

namespace JAFD
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
		ReturnCode spiNvsramSetup();

		// Read and write functions
		uint8_t readByte(uint32_t address);
		void writeByte(uint32_t address, uint8_t byte);
		void readPage(uint16_t numPage, uint8_t* buffer);
		void writePage(uint16_t numPage, uint8_t* buffer);
		void readStream(uint32_t address, uint8_t* buffer, uint32_t length);
		void writeStream(uint32_t address, uint8_t* buffer, uint32_t length);
	}
}