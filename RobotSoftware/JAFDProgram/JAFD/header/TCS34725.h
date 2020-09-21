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
	// Class for RGB Sensor
	class TCS34725
	{
	public:
		TCS34725(uint8_t channel, tcs34725IntegrationTime_t tcsIntegrationTime = TCS34725_INTEGRATIONTIME_700MS, tcs34725Gain_t tcsGain = TCS34725_GAIN_1X);
		ReturnCode setup();
		void updateSensorReadings();

	private:
		const uint8_t channel;
		Adafruit_TCS34725 sensor;
		uint16_t r, g, b, c, colorTemp, lux;
	};
}