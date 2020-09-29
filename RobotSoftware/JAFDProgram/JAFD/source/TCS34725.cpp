#include <Wire.h>

#include "../header/TCS34725.h"
#include "../../JAFDSettings.h"

namespace JAFD
{
	namespace ColorSensor
	{
		constexpr auto interruptPin = PinMapping::MappedPins[JAFDSettings::ColorSensor::interruptPin];
		constexpr auto tcsIntegrationTime = JAFDSettings::ColorSensor::tcsIntegrationTime;
		constexpr auto tcsGain = JAFDSettings::ColorSensor::tcsGain;

		Adafruit_TCS34725 sensor;
		volatile bool dataReady;
		
		ReturnCode setup()
		{
			dataReady = false;

			// Setup INT-Pin / Falling Edge Detection
			interruptPin.port->PIO_PER = interruptPin.pin;
			interruptPin.port->PIO_ODR = interruptPin.pin;
			interruptPin.port->PIO_PUER = interruptPin.pin;
			interruptPin.port->PIO_IER = interruptPin.pin;
			interruptPin.port->PIO_AIMER = interruptPin.pin;
			interruptPin.port->PIO_ESR = interruptPin.pin;
			interruptPin.port->PIO_FELLSR = interruptPin.pin;

			NVIC_EnableIRQ(static_cast<IRQn_Type>(interruptPin.portID));
			NVIC_SetPriority(static_cast<IRQn_Type>(interruptPin.portID), 1);

			volatile auto temp = interruptPin.port->PIO_ISR;

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
				dataReady = true;
			}
		}

		bool dataIsReady()
		{
			return dataReady;
		}

		void getData(uint16_t* colorTemp, uint16_t* lux)
		{
			if (dataReady)
			{
				uint16_t c = sensor.read16(TCS34725_CDATAL);
				uint16_t r = sensor.read16(TCS34725_RDATAL);
				uint16_t g = sensor.read16(TCS34725_GDATAL);
				uint16_t b = sensor.read16(TCS34725_BDATAL);

				*colorTemp = sensor.calculateColorTemperature(r, g, b);
				*lux = sensor.calculateLux(r, g, b);

				dataReady = false;
				sensor.clearInterrupt();
			}
			else
			{
				*colorTemp = (uint16_t)-1;
				*lux = (uint16_t)-1;
			}
		}
	}
}