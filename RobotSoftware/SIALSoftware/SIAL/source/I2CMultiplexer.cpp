#include <Wire.h>

#include "../header/I2CMultiplexer.h"
#include "../SIALSettings.h"
#include "../header/SmallThings.h"

namespace SIAL
{
	namespace I2CMultiplexer
	{
		namespace
		{
			uint8_t _currentChannel;
		}

		ReturnCode setup()
		{
			if (!checkI2C()) {
				I2C::recoverI2C();
			}

			uint8_t ch = random(0, maxCh + 1);
			_currentChannel = ch;

			if (selectChannel(ch) != 0) return ReturnCode::error;
			else return ReturnCode::ok;
		}

		bool checkI2C() {
			uint8_t ch = random(0, maxCh + 1);
			selectChannel(ch);
			Wire.requestFrom(0x70, 1, true);

			uint8_t b = Wire.read();

			if (b != 1 << ch) {
				Serial.println("I2C Multiplexer Error");
				return false;
			}

			return true;
		}

		uint8_t getChannel()
		{
			return _currentChannel;
		}

		uint8_t selectChannel(uint8_t channel)
		{
			if (channel >= 0 && channel <= maxCh)
			{
				Wire.beginTransmission(0x70);
				Wire.write(1 << channel);

				_currentChannel = channel;

				auto result = Wire.endTransmission(true);

				return result;
			}

			return 4;
		}
	}
}