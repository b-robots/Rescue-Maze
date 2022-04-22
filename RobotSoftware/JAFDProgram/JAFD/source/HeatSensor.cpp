/*
This part of the Library is responsible for heat sensors
*/

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include "../../JAFDSettings.h"
#include "../header/AllDatatypes.h"
#include "../header/HeatSensor.h"
#include "../header/TCA9548A.h"
#include <Adafruit_AMG88xx.h>
#include <TPA81.h>

#include <algorithm>
#include <functional>

#ifdef USE_AMG833
#error "AMG8833 doesn't work on Wire1"
#endif

namespace JAFD
{
	namespace HeatSensor
	{
		namespace
		{
#ifdef USE_AMG833
			Adafruit_AMG88xx amgLeft;
			Adafruit_AMG88xx amgRight;
#else
			TPA81 tpaLeft;
			TPA81 tpaRight;
#endif

			float ambientTemp = 23.0f;

			ReturnCode readAmbientTemp()
			{
#ifdef USE_AMG8833
				float pixels[AMG88xx_PIXEL_ARRAY_SIZE];

				amgLeft.readPixels(pixels);

				for (size_t i = 0; i < AMG88xx_PIXEL_ARRAY_SIZE; i++)
				{
					ambientTemp += pixels[i];
				}

				amgRight.readPixels(pixels);

				for (size_t i = 0; i < AMG88xx_PIXEL_ARRAY_SIZE; i++)
				{
					ambientTemp += pixels[i];
				}

				ambientTemp /= (2.0f * AMG88xx_PIXEL_ARRAY_SIZE);
				ambientTemp = (ambientTemp + amgLeft.readThermistor() + amgRight.readThermistor()) / 3.0f;
#else
				int pixels[8];

				I2CMultiplexer::selectChannel(JAFDSettings::HeatSensors::Left::i2cChannel);
				int leftAmbient = tpaLeft.getAll(pixels);
				if (leftAmbient == 0 || leftAmbient > 40) return ReturnCode::error;

				for (size_t i = 0; i < 8; i++)
				{
					ambientTemp += pixels[i];
				}

				I2CMultiplexer::selectChannel(JAFDSettings::HeatSensors::Right::i2cChannel);
				int rightAmbient = tpaRight.getAll(pixels);
				if (rightAmbient == 0 || rightAmbient > 40) return ReturnCode::error;

				for (size_t i = 0; i < 8; i++)
				{
					ambientTemp += pixels[i];
				}

				ambientTemp /= (2.0f * 8.0f);
				ambientTemp = (ambientTemp + fmaxf(leftAmbient, rightAmbient)) / 2.0f;
#endif
				ambientTemp = 23.0f;
				return ReturnCode::ok;
			}
		}

		ReturnCode reset()
		{
			return setup();
		}

		ReturnCode setup()
		{
#ifdef USE_AMG833
			if (!amgLeft.begin(JAFDSettings::HeatSensors::Left::i2cAddr) || !amgRight.begin(JAFDSettings::HeatSensors::Right::i2cAddr)) return ReturnCode::error;

			amgLeft.setMovingAverageMode(false);
			amgRight.setMovingAverageMode(false);
#else
#endif
			I2CMultiplexer::selectChannel(JAFDSettings::HeatSensors::Left::i2cChannel);
			tpaLeft.setup(0xD0);

			I2CMultiplexer::selectChannel(JAFDSettings::HeatSensors::Right::i2cChannel);
			tpaRight.setup(0xD0);

			return readAmbientTemp();
		}

		bool detectVictim(HeatSensorSide sensor)
		{
#ifdef USE_AMG833
			constexpr uint8_t numPix = AMG88xx_PIXEL_ARRAY_SIZE;
			float pixels[numPix];
#else
			constexpr uint8_t numPix = 8;
			int pixels[numPix];
#endif

			if (sensor == HeatSensorSide::left)
			{
#ifdef USE_AMG833
				amgLeft.readPixels(pixels);
#else
				I2CMultiplexer::selectChannel(JAFDSettings::HeatSensors::Left::i2cChannel);
				tpaLeft.getAll(pixels);
#endif
			}
			else
			{
#ifdef USE_AMG833
				amgLeft.readPixels(pixels);
#else
				I2CMultiplexer::selectChannel(JAFDSettings::HeatSensors::Right::i2cChannel);
				tpaRight.getAll(pixels);
#endif
			}

#ifdef USE_AMG833
			std::sort(pixels, pixels + numPix, std::greater<float>());
#else
			std::sort(pixels, pixels + numPix, std::greater<int>());
#endif
			float avgHigh = (pixels[0] * 0.6f + pixels[1] * 0.4f);

			if (sensor == HeatSensorSide::right) {
				avgHigh -= 3.0f;
			}

			if (avgHigh >= JAFDSettings::HeatSensors::threshold + ambientTemp) return true;
			else return false;
		}
	}
}

