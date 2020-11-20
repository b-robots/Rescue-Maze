#include <Wire.h>

#include "../header/TCA9548A.h"
#include "../../JAFDSettings.h"

namespace JAFD
{
	namespace I2CMultiplexer
	{
		namespace
		{
			constexpr uint8_t _maxCh = 8;
			uint8_t _currentChannel;
		}

		ReturnCode setup()
		{
			if (!selectChannel(0)) return ReturnCode::error;
			else return ReturnCode::ok;
		}

		uint8_t getChannel()
		{
			return _currentChannel;
		}

		uint8_t selectChannel(uint8_t channel)
		{
			if (channel >= 0 && channel < _maxCh)
			{
				Wire.beginTransmission(JAFDSettings::DistanceSensors::multiplexerAddr);
				Wire.write(1 << channel);

				_currentChannel = channel;
				return(Wire.endTransmission());
			}

			return 0;
		}
	}
}