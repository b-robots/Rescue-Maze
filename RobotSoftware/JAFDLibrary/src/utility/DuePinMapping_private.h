/*
This private file of the library is responsible for the access to the SPI EEPROM
*/

#pragma once

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

namespace JAFD
{
	namespace PinMapping
	{
		// Informations about one Pin on the SAM3x8e
		typedef struct PinInformation
		{
			uint32_t pin;
			Pio* port;
			uint8_t portID;
		};

		// Convert Pin from Arduino Numbering to SAM3x8e Pin
		constexpr PinInformation MappedPins[] = {
			{ PIO_PA8, PIOA, ID_PIOA },
			{ PIO_PA9, PIOA, ID_PIOA },
			{ PIO_PB25, PIOB, ID_PIOB },
			{ PIO_PC28, PIOC, ID_PIOC },
			{ PIO_PC26, PIOC, ID_PIOC },
			{ PIO_PC25, PIOC, ID_PIOC },
			{ PIO_PC24, PIOC, ID_PIOC },
			{ PIO_PC23, PIOC, ID_PIOC },
			{ PIO_PC22, PIOC, ID_PIOC },
			{ PIO_PC21, PIOC, ID_PIOC },
			{ PIO_PC29, PIOC, ID_PIOC },
			{ PIO_PD7, PIOD, ID_PIOD },
			{ PIO_PD8, PIOD, ID_PIOD },
			{ PIO_PB27, PIOB, ID_PIOB },
			{ PIO_PD4, PIOD, ID_PIOD },
			{ PIO_PD5, PIOD, ID_PIOD },
			{ PIO_PA13, PIOA, ID_PIOA },
			{ PIO_PA12, PIOA, ID_PIOA },
			{ PIO_PA11, PIOA, ID_PIOA },
			{ PIO_PA10, PIOA, ID_PIOA },
			{ PIO_PB12, PIOB, ID_PIOB },
			{ PIO_PB13, PIOB, ID_PIOB },
			{ PIO_PB26, PIOB, ID_PIOB },
			{ PIO_PA14, PIOA, ID_PIOA },
			{ PIO_PA15, PIOA, ID_PIOA },
			{ PIO_PD0, PIOD, ID_PIOD },
			{ PIO_PD1, PIOD, ID_PIOD },
			{ PIO_PD2, PIOD, ID_PIOD },
			{ PIO_PD3, PIOD, ID_PIOD },
			{ PIO_PD6, PIOD, ID_PIOD },
			{ PIO_PD9, PIOD, ID_PIOD },
			{ PIO_PA7, PIOA, ID_PIOA },
			{ PIO_PD10, PIOD, ID_PIOD },
			{ PIO_PC1, PIOC, ID_PIOC },
			{ PIO_PC2, PIOC, ID_PIOC },
			{ PIO_PC3, PIOC, ID_PIOC },
			{ PIO_PC4, PIOC, ID_PIOC },
			{ PIO_PC5, PIOC, ID_PIOC },
			{ PIO_PC6, PIOC, ID_PIOC },
			{ PIO_PC7, PIOC, ID_PIOC },
			{ PIO_PC8, PIOC, ID_PIOC },
			{ PIO_PC9, PIOC, ID_PIOC },
			{ PIO_PA19, PIOA, ID_PIOA },
			{ PIO_PA20, PIOA, ID_PIOA },
			{ PIO_PC19, PIOC, ID_PIOC },
			{ PIO_PC18, PIOC, ID_PIOC },
			{ PIO_PC17, PIOC, ID_PIOC },
			{ PIO_PC16, PIOC, ID_PIOC },
			{ PIO_PC15, PIOC, ID_PIOC },
			{ PIO_PC14, PIOC, ID_PIOC },
			{ PIO_PC13, PIOC, ID_PIOC },
			{ PIO_PC12, PIOC, ID_PIOC },
			{ PIO_PB21, PIOB, ID_PIOB },
			{ PIO_PB14, PIOB, ID_PIOB },
			{ PIO_PA16, PIOA, ID_PIOA },
			{ PIO_PA24, PIOA, ID_PIOA },
			{ PIO_PA23, PIOA, ID_PIOA },
			{ PIO_PA22, PIOA, ID_PIOA },
			{ PIO_PA6, PIOA, ID_PIOA },
			{ PIO_PA4, PIOA, ID_PIOA },
			{ PIO_PA3, PIOA, ID_PIOA },
			{ PIO_PA2, PIOA, ID_PIOA },
			{ PIO_PB17, PIOB, ID_PIOB },
			{ PIO_PB18, PIOB, ID_PIOB },
			{ PIO_PB19, PIOB, ID_PIOB },
			{ PIO_PB20, PIOB, ID_PIOB },
			{ PIO_PB15, PIOB, ID_PIOB },
			{ PIO_PB16, PIOB, ID_PIOB },
			{ PIO_PA1, PIOA, ID_PIOA },
			{ PIO_PA0, PIOA, ID_PIOA },
			{ PIO_PA17, PIOA, ID_PIOA },
			{ PIO_PA18, PIOA, ID_PIOA },
			{ PIO_PC30, PIOC, ID_PIOC },
			{ PIO_PA21, PIOA, ID_PIOA },
			{ PIO_PA25, PIOA, ID_PIOA },
			{ PIO_PA26, PIOA, ID_PIOA },
			{ PIO_PA27, PIOA, ID_PIOA },
			{ PIO_PA28, PIOA, ID_PIOA },
			{ PIO_PB23, PIOB, ID_PIOB }
		};		
	}
}