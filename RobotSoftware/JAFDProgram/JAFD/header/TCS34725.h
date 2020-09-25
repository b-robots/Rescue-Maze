#pragma once

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include "AllDatatypes.h"
#include <Adafruit_TCS34725.h>

namespace JAFD
{
	// TCS34725
	namespace ColorSensor
	{
		ReturnCode setup();
		void interrupt(const Interrupts::InterruptSource source, const uint32_t isr);
		bool dataIsReady();
		void getData(uint16_t* colorTemp, uint16_t* lux);
	}
}