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

		namespace {
			Adafruit_TCS34725 sensor;
			// volatile bool dataReady;
			float _whiteThresh = 300.0f;
			float _silverThresh = 90.0f;
			volatile uint32_t _lastRead;
		}
		
		ReturnCode setup()
		{
			//dataReady = false;

			//// Setup INT-Pin / Falling Edge Detection
			//interruptPin.port->PIO_PER = interruptPin.pin;
			//interruptPin.port->PIO_ODR = interruptPin.pin;
			//interruptPin.port->PIO_PUER = interruptPin.pin;
			//interruptPin.port->PIO_IER = interruptPin.pin;
			//interruptPin.port->PIO_AIMER = interruptPin.pin;
			//interruptPin.port->PIO_ESR = interruptPin.pin;
			//interruptPin.port->PIO_FELLSR = interruptPin.pin;

			sensor = Adafruit_TCS34725(tcsIntegrationTime, tcsGain);

			if (!sensor.begin(TCS34725_ADDRESS, &Wire1)) return ReturnCode::error;

			//sensor.write8(TCS34725_PERS, TCS34725_PERS_NONE);
			//sensor.setInterrupt(true);
			_lastRead = millis();

			return ReturnCode::ok;
		}

		void interrupt(const Interrupts::InterruptSource source, const uint32_t isr)
		{
			//if (interruptPin.portID == static_cast<uint8_t>(source) && (isr & interruptPin.pin))
			//{
			//	dataReady = true;
			//}
		}

		bool dataIsReady()
		{
			// DEBUG
			return (millis() - _lastRead) > 24;
		}

		void getData(uint16_t* colorTemp, uint16_t* lux)
		{
			uint16_t c = sensor.read16(TCS34725_CDATAL);
			uint16_t r = sensor.read16(TCS34725_RDATAL);
			uint16_t g = sensor.read16(TCS34725_GDATAL);
			uint16_t b = sensor.read16(TCS34725_BDATAL);

			*colorTemp = sensor.calculateColorTemperature(r, g, b);
			*lux = sensor.calculateLux(r, g, b);
			
			_lastRead = millis();

			//sensor.clearInterrupt();
			//dataReady = false;
		}

		void calibrate() {
			float whiteLux = 0.0f;
			float silverLux = 0.0f;
			float blackLux = 0.0f;

			while (Serial.available()) Serial.read();
			Serial.println("Set robot on white square!");
			while (!Serial.available());
			delay(10);
			while (Serial.available()) Serial.read();

			auto t = millis();
			uint32_t i = 0;
			while (millis() - t < 5000) {
				if (dataIsReady()) {
					uint16_t temp, lux;
					getData(&temp, &lux);
					whiteLux += lux;
					i++;
				}
			}

			if (i > 0) {
				whiteLux /= i;
			}
			else {
				whiteLux = 350.0f;
			}

			while (Serial.available()) Serial.read();
			Serial.println("Set robot on silver square!");
			while (!Serial.available());
			delay(10);
			while (Serial.available()) Serial.read();

			t = millis();
			i = 0;
			while (millis() - t < 5000) {
				if (dataIsReady()) {
					uint16_t temp, lux;
					getData(&temp, &lux);
					silverLux += lux;
					i++;
				}
			}

			if (i > 0) {
				silverLux /= i;
			}
			else {
				silverLux = 200.0f;
			}

			while (Serial.available()) Serial.read();
			Serial.println("Set robot on black square!");
			while (!Serial.available());
			delay(10);
			while (Serial.available()) Serial.read();

			t = millis();
			i = 0;
			while (millis() - t < 5000) {
				if (dataIsReady()) {
					uint16_t temp, lux;
					getData(&temp, &lux);
					blackLux += lux;
					i++;
				}
			}

			if (i > 0) {
				blackLux /= i;
			}
			else {
				blackLux = 30.0f;
			}

			Serial.println("w: " + String(whiteLux) + ", s: " + String(silverLux) + ", b: " + String(blackLux));

			_whiteThresh = (whiteLux + silverLux) / 2.0f;
			_silverThresh = (blackLux + silverLux) / 2.0f;
		}

		FloorTileColour detectTileColour(uint16_t lux) {
			if (lux > _whiteThresh) {
				return FloorTileColour::white;
			}
			else if (lux > _silverThresh) {
				return FloorTileColour::silver;
			}
			else if (lux > 5) {
				return FloorTileColour::black;
			}

			return FloorTileColour::white;
		}
	}
}