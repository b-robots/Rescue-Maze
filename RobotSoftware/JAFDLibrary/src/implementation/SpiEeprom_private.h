/*
This private file of the library is responsible for the access to the SPI EEPROM
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
#include "SpiEeprom_public.h"

namespace JAFD
{
	// Namespace for the SPI EEPROM "25LC1024"
	// The written data is stored inverted in the EEPROM -> default value is '0'
	namespace SpiEeprom
	{
		// Helping functions
		void enable();
		void disable();

		// Devise specific constants
		static constexpr uint8_t pageSize = 256;
		static constexpr uint8_t sectorSize = 32000;

		// Init
		ReturnCode spiEepromSetup(SpiEepromSet set);

		// Read and write functions
		uint8_t readByte(uint32_t address);
		void writeByte(uint32_t address, uint8_t byte);
		void readPage(uint8_t numPage, uint8_t* buffer);
		void writePage(uint8_t numPage, uint8_t* buffer);
		void readStream(uint32_t address, uint8_t* buffer, uint32_t length);
		void writeStream(uint32_t address, uint8_t* buffer, uint32_t length);

		// Erase functions
		void erasePage(uint8_t numPage);
		void eraseSector(uint8_t numSector);
		void eraseChip();
	}
}