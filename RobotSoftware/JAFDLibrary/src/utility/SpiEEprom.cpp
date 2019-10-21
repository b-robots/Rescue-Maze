/*
This private file of the library is responsible for the access to the SPI EEPROM
*/

#include "SpiEeprom_private.h"
#include "DuePinMapping_private.h"

#include <SPI.h>

namespace JAFD
{
	void SpiEeprom::init(uint8_t ssPin)
	{
		// Setup SS Pin
		_ssPin = ssPin;

		PMC->PMC_PCER0 = 1 <<  PinMapping::MappedPins[_ssPin].portID;
		PinMapping::MappedPins[_ssPin].port->PIO_PER = PinMapping::MappedPins[_ssPin].pin;
		PinMapping::MappedPins[_ssPin].port->PIO_OER = PinMapping::MappedPins[_ssPin].pin;
	}

	void SpiEeprom::enable()
	{
		PinMapping::MappedPins[_ssPin].port->PIO_CODR = PinMapping::MappedPins[_ssPin].pin;
	}

	void SpiEeprom::disable()
	{
		PinMapping::MappedPins[_ssPin].port->PIO_SODR = PinMapping::MappedPins[_ssPin].pin;
	}

	uint8_t SpiEeprom::readByte(uint32_t address)
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

	void SpiEeprom::writeByte(uint32_t address, uint8_t byte)
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

	void SpiEeprom::readPage(uint8_t numPage, uint8_t* buffer)
	{
		// Read Data
		enable();

		SPI.transfer((uint8_t)Instruction::read);

		uint32_t address = numPage * pageSize;

		SPI.transfer((uint8_t)(address >> 16));
		SPI.transfer((uint8_t)(address >> 8));
		SPI.transfer((uint8_t)(address));

		for (uint8_t i = 0; i < pageSize; i++)
		{
			*(buffer++) = ~SPI.transfer(0x00);
		}

		disable();
	}

	void SpiEeprom::writePage(uint8_t numPage, uint8_t* buffer)
	{
		// Set Write Enable Bit
		enable();
		SPI.transfer((uint8_t)Instruction::wren);
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

	void SpiEeprom::readStream(uint32_t address, uint8_t* buffer, uint32_t length)
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

	void SpiEeprom::writeStream(uint32_t address, uint8_t* buffer, uint32_t length)
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

		for (uint32_t i = 0; i < length; i++)
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

	void SpiEeprom::erasePage(uint8_t numPage)
	{
		// Set Write Enable Bit
		enable();
		SPI.transfer((uint8_t)Instruction::wren);
		disable();

		// Erase Page
		enable();

		SPI.transfer((uint8_t)Instruction::pe);

		uint32_t address = numPage * pageSize;


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

	void SpiEeprom::eraseSector(uint8_t numSector)
	{
		// Set Write Enable Bit
		enable();
		SPI.transfer((uint8_t)Instruction::wren);
		disable();

		// Erase Page
		enable();

		SPI.transfer((uint8_t)Instruction::se);

		uint32_t address = numSector * sectorSize;


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

	void SpiEeprom::eraseChip()
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