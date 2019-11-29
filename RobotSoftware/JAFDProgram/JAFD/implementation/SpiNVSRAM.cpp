/*
This private file of the library is responsible for the access to the SPI EEPROM
*/

#include "../../JAFDSettings.h"
#include "SpiNVSRAM_private.h"
#include "../utility/DuePinMapping_private.h"

#include <SPI.h>

namespace JAFD
{
	namespace SpiNVSRAM
	{
		namespace
		{
			enum class Instruction : uint8_t
			{
				read = 0b00000011,	// Read data from memory array beginning at selected address
				write = 0b00000010, // Write data to memory array beginning at selected address
				edio = 0b00111011,	// Enter Dual I/O access
				rstio = 0b11111111, // Reset Dual I/O access
				rdmr = 0b00000101,	// Read Mode Register
				wrmr = 0b00000001,	// Write Mode Register 
			};

			constexpr auto _ssPin = PinMapping::MappedPins[JAFDSettings::SpiEeprom::ssPin];		// Slave-Select Pin
		}

		ReturnCode spiNvsramSetup()
		{
			_ssPin.port->PIO_PER = _ssPin.pin;
			_ssPin.port->PIO_OER = _ssPin.pin;

			return ReturnCode::ok;
		}

		void enable()
		{
			_ssPin.port->PIO_CODR = _ssPin.pin;
		}

		void disable()
		{
			_ssPin.port->PIO_SODR = _ssPin.pin;
		}

		uint8_t readByte(uint32_t address)
		{
			// Set Byte Mode
			enable();

			SPI.transfer((uint8_t)Instruction::wrmr);
			SPI.transfer(0b00000000);

			disable();

			// Read Data
			enable();

			SPI.transfer((uint8_t)Instruction::read);

			SPI.transfer((uint8_t)(address >> 16));
			SPI.transfer((uint8_t)(address >> 8));
			SPI.transfer((uint8_t)(address));

			auto val = SPI.transfer(0x00);

			disable();

			return val;
		}

		void writeByte(uint32_t address, uint8_t byte)
		{
			// Set Byte Mode
			enable();

			SPI.transfer((uint8_t)Instruction::wrmr);
			SPI.transfer(0b00000000);

			disable();

			// Write Data
			enable();

			SPI.transfer((uint8_t)Instruction::write);

			SPI.transfer((uint8_t)(address >> 16));
			SPI.transfer((uint8_t)(address >> 8));
			SPI.transfer((uint8_t)(address));

			SPI.transfer(byte);

			disable();
		}

		void readPage(uint16_t numPage, uint8_t* buffer)
		{
			// Set Page Mode
			enable();

			SPI.transfer((uint8_t)Instruction::wrmr);
			SPI.transfer(0b10000000);

			disable();

			// Read Data
			enable();

			SPI.transfer((uint8_t)Instruction::read);

			uint32_t address = numPage * pageSize;

			SPI.transfer((uint8_t)(address >> 16));
			SPI.transfer((uint8_t)(address >> 8));
			SPI.transfer((uint8_t)(address));

			for (uint8_t i = 0; i < pageSize; i++)
			{
				*(buffer++) = SPI.transfer(0x00);
			}

			disable();
		}

		void writePage(uint16_t numPage, uint8_t* buffer)
		{
			// Set Page Mode
			enable();

			SPI.transfer((uint8_t)Instruction::wrmr);
			SPI.transfer(0b10000000);

			disable();

			// Write Data
			enable();

			SPI.transfer((uint8_t)Instruction::write);

			uint32_t address = numPage * pageSize;

			SPI.transfer((uint8_t)(address >> 16));
			SPI.transfer((uint8_t)(address >> 8));
			SPI.transfer((uint8_t)(address));

			for (uint8_t i = 0; i < pageSize; i++)
			{
				SPI.transfer(*(buffer++));
			}

			disable();
		}

		void readStream(uint32_t address, uint8_t* buffer, uint32_t length)
		{
			// Set Stream Mode
			enable();

			SPI.transfer((uint8_t)Instruction::wrmr);
			SPI.transfer(0b01000000);

			disable();

			// Read Data
			enable();

			SPI.transfer((uint8_t)Instruction::read);

			SPI.transfer((uint8_t)(address >> 16));
			SPI.transfer((uint8_t)(address >> 8));
			SPI.transfer((uint8_t)(address));

			for (uint16_t i = 0; i < length; i++)
			{
				*(buffer++) = SPI.transfer(0x00);
			}

			disable();
		}

		void writeStream(uint32_t address, uint8_t* buffer, uint32_t length)
		{
			// Set Stream Mode
			enable();

			SPI.transfer((uint8_t)Instruction::wrmr);
			SPI.transfer(0b01000000);

			disable();

			// Write Data
			enable();

			SPI.transfer((uint8_t)Instruction::write);

			SPI.transfer((uint8_t)(address >> 16));
			SPI.transfer((uint8_t)(address >> 8));
			SPI.transfer((uint8_t)(address));

			for (uint32_t i = 0; i < length; i++)
			{
				SPI.transfer(*(buffer++));

				// If Page-Overflow initiate new write cycle
				if (++address % pageSize == 0)
				{
					disable();
					enable();

					// Write address
					SPI.transfer((uint8_t)Instruction::read);

					SPI.transfer((uint8_t)(address >> 16));
					SPI.transfer((uint8_t)(address >> 8));
					SPI.transfer((uint8_t)(address));
				}
			}

			disable();
		}
	}
}