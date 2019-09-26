/*
This private file of the library is responsible for the access to the SPI EEPROM
*/

#include "SpiEeprom_private.h"

namespace JAFD
{
	namespace SpiEeprom
	{
		SpiEeprom::SpiEeprom(uint8_t ssPin) : _ssPin(PinMapping::MappedPins[ssPin])
		{
			_ssPin.port->PIO_OER = _ssPin.pin;
		}

		void SpiEeprom::enable()
		{
			_ssPin.port->PIO_SODR = _ssPin.pin;
		}

		void SpiEeprom::disable()
		{
			_ssPin.port->PIO_CODR = _ssPin.pin;
		}

		uint8_t SpiEeprom::readByte(uint32_t address)
		{
			enable();
			SPI.transfer((uint8_t)Instruction::read);

			SPI.transfer((uint8_t)(address >> 16));
			SPI.transfer((uint8_t)(address >> 8));
			SPI.transfer((uint8_t)(address));
		}
	}
}