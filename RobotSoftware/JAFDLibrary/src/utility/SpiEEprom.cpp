/*
This private file of the library is responsible for the access to the SPI EEPROM
*/

#include "SpiEeprom_private.h"
#include <SPI.h>

namespace JAFD
{
	void SpiEeprom::init(uint8_t ssPin)
	{
		_ssPin = PinMapping::MappedPins[ssPin];

		PMC->PMC_PCER0 = 1 << _ssPin.portID;
		_ssPin.port->PIO_PER = _ssPin.pin;
		_ssPin.port->PIO_OER = _ssPin.pin;

		enable();

		SPI.transfer((uint8_t)Instruction::wren);

		disable();
	}

	void SpiEeprom::enable()
	{
		_ssPin.port->PIO_CODR = _ssPin.pin;
	}

	void SpiEeprom::disable()
	{
		_ssPin.port->PIO_SODR = _ssPin.pin;
	}

	uint8_t SpiEeprom::readByte(uint32_t address)
	{
		enable();

		SPI.transfer((uint8_t)Instruction::read);

		SPI.transfer((uint8_t)(address >> 16));
		SPI.transfer((uint8_t)(address >> 8));
		SPI.transfer((uint8_t)(address));

		auto val = SPI.transfer(0x00);

		disable();

		return val;
	}

	void SpiEeprom::writeByte(uint32_t address, uint8_t byte)
	{
		enable();

		SPI.transfer((uint8_t)Instruction::write);

		SPI.transfer((uint8_t)(address >> 16));
		SPI.transfer((uint8_t)(address >> 8));
		SPI.transfer((uint8_t)(address));

		SPI.transfer(byte);

		disable();
	}

	void SpiEeprom::readPage(uint8_t numPage, uint8_t* buffer)
	{
		enable();

		SPI.transfer((uint8_t)Instruction::read);

		uint32_t address = numPage * _pageSize;

		SPI.transfer((uint8_t)(address >> 16));
		SPI.transfer((uint8_t)(address >> 8));
		SPI.transfer((uint8_t)(address));

		for (uint8_t i = 0; i < _pageSize; i++)
		{
			*buffer++ = SPI.transfer(0x00);
		}

		disable();
	}

	void SpiEeprom::writePage(uint8_t numPage, uint8_t* buffer)
	{
		enable();

		SPI.transfer((uint8_t)Instruction::write);

		uint32_t address = numPage * _pageSize;

		SPI.transfer((uint8_t)(address >> 16));
		SPI.transfer((uint8_t)(address >> 8));
		SPI.transfer((uint8_t)(address));

		for (uint8_t i = 0; i < _pageSize; i++)
		{
			SPI.transfer(*buffer++);
		}

		disable();
	}

	void SpiEeprom::readStream(uint32_t address, uint8_t* buffer, uint32_t length)
	{
		enable();

		SPI.transfer((uint8_t)Instruction::read);

		SPI.transfer((uint8_t)(address >> 16));
		SPI.transfer((uint8_t)(address >> 8));
		SPI.transfer((uint8_t)(address));

		for (uint16_t i = 0; i < length; i++)
		{
			*buffer++ = SPI.transfer(0x00);

			if (++address % 256 == 0)
			{
				disable();

				for (volatile uint8_t d = 0; d < 4; d++);

				enable();

				SPI.transfer((uint8_t)Instruction::read);

				SPI.transfer((uint8_t)(address >> 16));
				SPI.transfer((uint8_t)(address >> 8));
				SPI.transfer((uint8_t)(address));
			}
		}

		disable();
	}

	void SpiEeprom::writeStream(uint32_t address, uint8_t* buffer, uint32_t length)
	{
		enable();

		SPI.transfer((uint8_t)Instruction::write);

		SPI.transfer((uint8_t)(address >> 16));
		SPI.transfer((uint8_t)(address >> 8));
		SPI.transfer((uint8_t)(address));

		for (uint16_t i = 0; i < length; i++)
		{
			SPI.transfer(*buffer++);

			if (++address % 256 == 0)
			{
				disable();

				for (volatile uint8_t d = 0; d < 4; d++);

				enable();

				SPI.transfer((uint8_t)Instruction::read);

				SPI.transfer((uint8_t)(address >> 16));
				SPI.transfer((uint8_t)(address >> 8));
				SPI.transfer((uint8_t)(address));
			}
		}

		disable();
	}
	
}