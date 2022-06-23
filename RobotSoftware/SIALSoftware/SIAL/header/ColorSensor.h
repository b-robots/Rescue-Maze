#pragma once

#include "AllDatatypes.h"
#include <Adafruit_TCS34725.h>

namespace SIAL
{
	// TCS34725
	namespace ColorSensor
	{
		ReturnCode setup();
		bool dataIsReady();
		void getData(uint16_t* colorTemp, uint16_t* lux);
		void calibrate();
		FloorTileColour detectTileColour(uint16_t lux);
	}
}