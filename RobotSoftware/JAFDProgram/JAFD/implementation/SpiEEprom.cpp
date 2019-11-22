/*
This private file of the library is responsible for the access to the SPI EEPROM
*/

#include "../../JAFDSettings.h"
#include "SpiEeprom_private.h"
#include "../utility/DuePinMapping_private.h"

#include <SPI.h>

namespace JAFD
{
	namespace SpiEeprom
	{
		namespace
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

			constexpr uint8_t _ssPin = JAFDSettings::SpiEeprom::ssPin;	// Slave-Select Pin
		}

		ReturnCode spiEepromSetup()
		{
			PMC->PMC_PCER0 = 1 << PinMapping::MappedPins[_ssPin].portID;
			PinMapping::MappedPins[_ssPin].port->PIO_PER = PinMapping::MappedPins[_ssPin].pin;
			PinMapping::MappedPins[_ssPin].port->PIO_OER = PinMapping::MappedPins[_ssPin].pin;

			return ReturnCode::ok;
		}

		void enable()
		{
			PinMapping::MappedPins[_ssPin].port->PIO_CODR = PinMapping::MappedPins[_ssPin].pin;
		}

		void disable()
		{
			PinMapping::MappedPins[_ssPin].port->PIO_SODR = PinMapping::MappedPins[_ssPin].pin;
		}

		uint8_t readByte(unsigned int address)
		{
			// Read Data
			enable();

			SPI.transfer((uint8_t)Instruction::read);

			SPI.transfer((uint8_t)(address >> 16));
			SPI.transfer((uint8_t)(address >> 8));
			SPI.transfer((uint8_t)(address));

			auto val = ~SPI.transfer(0x00);

			disable();

			return val;
		}

		void writeByte(unsigned int address, uint8_t byte)
		{
			// Set Write Enable Bit
			enable();
			SPI.transfer((uint8_t)Instruction::wren);
			disable();

			// Write Data
			enable();

			SPI.transfer((uint8_t)Instruction::write);

			SPI.transfer((uint8_t)(address >> 16));
			SPI.transfer((uint8_t)(address >> 8));
			SPI.transfer((uint8_t)(address));

			SPI.transfer(~byte);

			disable();

			// Wait for end of write
			enable();
			do
			{
				SPI.transfer((uint8_t)Instruction::rdsr);
			} while (SPI.transfer(0xff) & 1);
			disable();
		}

		void readPage(uint16_t numPage, uint8_t* buffer)
		{
			// Read Data
			enable();

			SPI.transfer((uint8_t)Instruction::read);

			unsigned int address = numPage * pageSize;

			SPI.transfer((uint8_t)(address >> 16));
			SPI.transfer((uint8_t)(address >> 8));
			SPI.transfer((uint8_t)(address));

			for (uint8_t i = 0; i < pageSize; i++)
			{
				*(buffer++) = ~SPI.transfer(0x00);
			}

			disable();
		}

		void writePage(uint16_t numPage, uint8_t* buffer)
		{
			// Set Write Enable Bit
			enable();
			SPI.transfer((uint8_t)Instruction::wren);
			disable();

			// Write Data
			enable();

			SPI.transfer((uint8_t)Instruction::write);

			unsigned int address = numPage * pageSize;

			SPI.transfer((uint8_t)(address >> 16));
			SPI.transfer((uint8_t)(address >> 8));
			SPI.transfer((uint8_t)(address));

			for (uint8_t i = 0; i < pageSize; i++)
			{
				SPI.transfer(~(*(buffer++)));
			}

			disable();

			// Wait for end of write
			enable();
			do
			{
				SPI.transfer((uint8_t)Instruction::rdsr);
			} while (SPI.transfer(0xff) & 1);
			disable();
		}

		void readStream(unsigned int address, uint8_t* buffer, unsigned int length)
		{
			// Read Data
			enable();

			SPI.transfer((uint8_t)Instruction::read);

			SPI.transfer((uint8_t)(address >> 16));
			SPI.transfer((uint8_t)(address >> 8));
			SPI.transfer((uint8_t)(address));

			for (uint16_t i = 0; i < length; i++)
			{
				*(buffer++) = ~SPI.transfer(0x00);
			}

			disable();
		}

		void writeStream(unsigned int address, uint8_t* buffer, unsigned int length)
		{
			// Set Write Enable Bit
			enable();
			SPI.transfer((uint8_t)Instruction::wren);
			disable();

			// Write Data
			enable();

			SPI.transfer((uint8_t)Instruction::write);

			SPI.transfer((uint8_t)(address >> 16));
			SPI.transfer((uint8_t)(address >> 8));
			SPI.transfer((uint8_t)(address));

			for (unsigned int i = 0; i < length; i++)
			{
				SPI.transfer(~(*(buffer++)));

				// If Page-Overflow initiate new write cycle
				if (++address % pageSize == 0)
				{
					// Wait for end of write
					enable();
					do
					{
						SPI.transfer((uint8_t)Instruction::rdsr);
					} while (SPI.transfer(0xff) & 1);

					// Set Write Enable Bit
					SPI.transfer((uint8_t)Instruction::wren);
					disable();

					// Write address
					enable();

					SPI.transfer((uint8_t)Instruction::read);

					SPI.transfer((uint8_t)(address >> 16));
					SPI.transfer((uint8_t)(address >> 8));
					SPI.transfer((uint8_t)(address));
				}
			}

			disable();

			// Wait for end of write
			enable();
			do
			{
				SPI.transfer((uint8_t)Instruction::rdsr);
			} while (SPI.transfer(0xff) & 1);
			disable();
		}

		void erasePage(uint16_t numPage)
		{
			// Set Write Enable Bit
			enable();
			SPI.transfer((uint8_t)Instruction::wren);
			disable();

			// Erase Page
			enable();

			SPI.transfer((uint8_t)Instruction::pe);

			unsigned int address = numPage * pageSize;


			SPI.transfer((uint8_t)(address >> 16));
			SPI.transfer((uint8_t)(address >> 8));
			SPI.transfer((uint8_t)(address));

			disable();

			// Wait for end of write
			enable();
			do
			{
				SPI.transfer((uint8_t)Instruction::rdsr);
			} while (SPI.transfer(0xff) & 1);
			disable();
		}

		void eraseSector(uint8_t numSector)
		{
			// Set Write Enable Bit
			enable();
			SPI.transfer((uint8_t)Instruction::wren);
			disable();

			// Erase Page
			enable();

			SPI.transfer((uint8_t)Instruction::se);

			unsigned int address = numSector * sectorSize;


			SPI.transfer((uint8_t)(address >> 16));
			SPI.transfer((uint8_t)(address >> 8));
			SPI.transfer((uint8_t)(address));

			disable();

			// Wait for end of write
			enable();
			do
			{
				SPI.transfer((uint8_t)Instruction::rdsr);
			} while (SPI.transfer(0xff) & 1);
			disable();
		}

		void eraseChip()
		{
			// Set Write Enable Bit
			enable();
			SPI.transfer((uint8_t)Instruction::wren);
			disable();

			// Erase Page
			enable();

			SPI.transfer((uint8_t)Instruction::ce);

			disable();

			// Wait for end of write
			enable();
			do
			{
				SPI.transfer((uint8_t)Instruction::rdsr);
			} while (SPI.transfer(0xff) & 1);
			disable();
		}
	}
}