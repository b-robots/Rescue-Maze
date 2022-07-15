#include <Arduino.h>

#include "../SIALSettings.h"
#include "../header/AllDatatypes.h"
#include "../header/HeatSensor.h"
#include "../header/I2CMultiplexer.h"

#include <TPA81.h>

#include <algorithm>
#include <functional>

namespace SIAL
{
	namespace HeatSensor
	{
		namespace
		{
			TPA81 tpaLeft;
			TPA81 tpaRight;

			float ambientTemp = 24.0f;

			ReturnCode readAmbientTemp()
			{
				int pixels[8];

				I2CMultiplexer::selectChannel(SIALSettings::HeatSensors::Left::i2cChannel);
				int leftAmbient = tpaLeft.getAll(pixels);
				if (leftAmbient == 0 || leftAmbient > 40) return ReturnCode::error;

				for (size_t i = 0; i < 8; i++)
				{
					ambientTemp += pixels[i];
				}

				I2CMultiplexer::selectChannel(SIALSettings::HeatSensors::Right::i2cChannel);
				int rightAmbient = tpaRight.getAll(pixels);
				if (rightAmbient == 0 || rightAmbient > 40) return ReturnCode::error;

				for (size_t i = 0; i < 8; i++)
				{
					ambientTemp += pixels[i];
				}

				ambientTemp /= (2.0f * 8.0f);
				ambientTemp = (ambientTemp + fmaxf(leftAmbient, rightAmbient)) / 2.0f;

				if (ambientTemp > 50.0f) {
					return ReturnCode::error;
				}

				// TESTING
				ambientTemp = 24.0f;
				return ReturnCode::ok;
			}
		}

		ReturnCode setup()
		{
			I2CMultiplexer::selectChannel(SIALSettings::HeatSensors::Left::i2cChannel);
			tpaLeft.setup(0xD0);

			I2CMultiplexer::selectChannel(SIALSettings::HeatSensors::Right::i2cChannel);
			tpaRight.setup(0xD0);

			return readAmbientTemp();
		}

		bool detectVictim(HeatSensorSide sensor)
		{
			constexpr uint8_t numPix = 8;
			int pixels[numPix];

			if (sensor == HeatSensorSide::left)
			{
				I2CMultiplexer::selectChannel(SIALSettings::HeatSensors::Left::i2cChannel);
				tpaLeft.getAll(pixels);
			}
			else
			{
				I2CMultiplexer::selectChannel(SIALSettings::HeatSensors::Right::i2cChannel);
				tpaRight.getAll(pixels);
			}

			std::sort(pixels, pixels + numPix, std::greater<int>());

			float avgHigh = (pixels[0] * 0.6f + pixels[1] * 0.4f);

			if (sensor == HeatSensorSide::right) {
				avgHigh -= 4.0f;
			}

			if (avgHigh > 60.0f) return false;

			//Serial.print(sensor == HeatSensorSide::right ? "right: " : "left: ");
			//Serial.println(avgHigh);

			if (avgHigh >= SIALSettings::HeatSensors::threshold + ambientTemp) return true;
			else return false;
		}
	}
}

