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
			read = 0b00000011,
			write = 0b00000010,
			wren = 0b00000110,
			wrdi = 0b00000100,
			rdsr = 0b00000101,
			wrsr = 0b00000001,
			pe = 0b01000010,
			se = 0b11011000,
			ce = 0b11000111,
			rdid = 0b10101011,
			dpd = 0b10111001
		};

		class SpiEeprom
		{
		private:
			PinMapping::PinInformation _ssPin;

			// Helping functions
			void enable();
			void disable();

		public:
			SpiEeprom(uint8_t ssPin);

			// Read and write functions
			uint8_t readByte(uint32_t address);
			void writeByte(uint32_t address, uint8_t byte);
			void readPage(uint8_t numPage, uint8_t* buffer);
			void writePage(uint8_t numPage, uint8_t* buffer);
			void readStream(uint32_t address, uint8_t* buffer, uint16_t length);
			void writeStream(uint32_t address, uint8_t* buffer, uint16_t length);
		};
	}
}