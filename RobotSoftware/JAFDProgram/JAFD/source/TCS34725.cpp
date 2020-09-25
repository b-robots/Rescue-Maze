#include <Wire.h>

#include "../header/TCS34725.h"

namespace JAFD
{
	namespace ColorSensor
	{
		constexpr auto interruptPin = PinMapping::MappedPins[JAFDSettings::ColorSensor::interruptPin];
		constexpr auto tcsIntegrationTime = JAFDSettings::ColorSensor::tcsIntegrationTime;
		constexpr auto tcsGain = JAFDSettings::ColorSensor::tcsGain;

		Adafruit_TCS34725 sensor;
		uint16_t r, g, b, c, dataReady;
		
		ReturnCode setup()
		{
			dataReady = false;

			sensor = Adafruit_TCS34725(tcsIntegrationTime, tcsGain);

			if (!sensor.begin(TCS34725_ADDRESS, &Wire1)) return ReturnCode::error;

			sensor.write8(TCS34725_PERS, TCS34725_PERS_NONE);
			sensor.setInterrupt(true);

			return ReturnCode::ok;
		}

		void interrupt(const Interrupts::InterruptSource source, const uint32_t isr)
		{
			if (interruptPin.portID == static_cast<uint8_t>(source) && (isr & interruptPin.pin))
			{
				c = sensor.read16(TCS34725_CDATAL);
				r = sensor.read16(TCS34725_RDATAL);
				g = sensor.read16(TCS34725_GDATAL);
				b = sensor.read16(TCS34725_BDATAL);

				dataReady = true;
			}
		}

		bool dataIsReady()
		{
			return dataReady;
		}

		void getData(uint16_t* colorTemp, uint16_t* lux)
		{
			*colorTemp = sensor.calculateColorTemperature(r, g, b);
			*lux = sensor.calculateLux(r, g, b);

			dataReady = false;
			sensor.clearInterrupt();
		}
	}
}