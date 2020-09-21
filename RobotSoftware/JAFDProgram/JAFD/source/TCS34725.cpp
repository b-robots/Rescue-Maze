#include <Wire.h>

#include "../header/TCS34725.h"

namespace JAFD
{
	TCS34725::TCS34725(uint8_t channel, tcs34725IntegrationTime_t tcsIntegrationTime, tcs34725Gain_t tcsGain) : channel(channel) 
	{
		sensor = Adafruit_TCS34725(tcsIntegrationTime, tcsGain);
	}

	ReturnCode TCS34725::setup()
	{
		if (sensor.begin()) return ReturnCode::ok;
		else return ReturnCode::error;
	}

	void TCS34725::updateSensorReadings()
	{
		//sensor.getRawData();
	}
}