#include <Wire.h>

#include "../header/TCA9548A.h"
#include "../../JAFDSettings.h"

namespace JAFD
{
	namespace I2CMultiplexer
	{
		namespace
		{
			uint8_t _currentChannel;
		}
		
		ReturnCode setup()
		{
			uint8_t ch = random(0, maxCh + 1);
			_currentChannel = ch;

			if (selectChannel(ch) != 0) return ReturnCode::error;
			else return ReturnCode::ok;
		}

		uint8_t getChannel()
		{
			return _currentChannel;
		}

		uint8_t selectChannel(uint8_t channel)
		{
			if (channel >= 0 && channel < maxCh)
			{
				Wire.beginTransmission(0x70);
				Wire.write(1 << channel);

				_currentChannel = channel;
				return(Wire.endTransmission(true));
			}

			return 4;
		}
	}
}