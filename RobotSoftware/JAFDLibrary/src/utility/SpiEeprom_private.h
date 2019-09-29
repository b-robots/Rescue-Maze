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

#include "../utility/DuePinMapping_private.h"

namespace JAFD
{
	namespace SpiEeprom
	{
		enum class Instruction : uint8_t
		{
			read = 0b00000011, // Read data from memory array beginning at selected address
			write = 0b00000010, // Write data to memory array beginning at selected address
			wren = 0b00000110, // Set the write enable latch (enable write operations)
			wrdi = 0b00000100, // Reset the write enable latch (disable write operations)
			rdsr = 0b00000101, // Read STATUS register
			wrsr = 0b00000001, // Write STATUS register 
			pe = 0b01000010, // Page Erase – erase one page in memory array
			se = 0b11011000, // Sector Erase – erase one sector in memory array
			ce = 0b11000111, // Chip Erase – erase all sectors in memory array
			rdid = 0b10101011, // Release from Deep power-down and read electronic signature
			dpd = 0b10111001 // Deep Power-Down mode
		};

		// Class for the SPI EEPROM "25LC1024"
		class Eeprom25LC1024
		{
		private:
			PinMapping::PinInformation _ssPin;

			// Helping functions
			void enable();
			void disable();

			static const uint8_t _pageSize = 256;

		public:
			Eeprom25LC1024(uint8_t ssPin);
			Eeprom25LC1024();

			// Read and write functions
			uint8_t readByte(uint32_t address);
			void writeByte(uint32_t address, uint8_t byte);
			void readPage(uint8_t numPage, uint8_t* buffer);
			void writePage(uint8_t numPage, uint8_t* buffer);
			void readStream(uint32_t address, uint8_t* buffer, uint32_t length);
			void writeStream(uint32_t address, uint8_t* buffer, uint32_t length);
		};
	}
}