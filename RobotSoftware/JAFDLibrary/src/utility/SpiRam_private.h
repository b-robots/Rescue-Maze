/*
This private file of the library is responsible for the access to the SPI RAM
*/

#pragma once

#include <SPI.h>
#include <stdint.h>

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

namespace JAFD
{
	namespace SpiRam
	{
		enum class OperationMode : uint8_t
		{
			byte = 0b00000000,
			page = 0b10000000,
			sequential = 0b01000000
		};

		enum class Instruction : uint8_t
		{
			read = 0x03,
			write = 0x02,
			esdi = 0x3b,
			esqi = 0x38,
			rstdqi = 0xff,
			rdmr = 0x05,
			wrmr = 0x01
		};

		class SpiRam
		{
		private:
			byte ssPin;
			OperationMode currentMode;

			// Helping functions
			void enable();
			void disable();
			void setMode(OperationMode mode);

		public:
			SpiRam(byte ssPin);

			// Read and write functions
			uint8_t readByte(uint32_t address);
			void writeByte(uint32_t address, uint8_t byte);
			void readPage(uint32_t address, uint8_t* buffer);
			void writePage(uint32_t address, uint8_t* buffer);
			void readStream(uint32_t address, uint8_t* buffer, uint16_t length);
			void writeStream(uint32_t address, uint8_t* buffer, uint16_t length);
		};
	}
}