#include <Wire.h>

#include "../header/TCA9548A.h"

namespace JAFD
{
	TCA9548A::TCA9548A(uint8_t address) :_address(address) {}

	uint8_t TCA9548A::getChannel() const
	{
		return _currentChannel;
	}

	uint8_t TCA9548A::selectChannel(uint8_t channel) 
	{
		if (channel >= 0 && channel < _maxCh)
		{
			Wire.beginTransmission(_address);
			Wire.write(1 << channel);

			_currentChannel = channel;
			return(Wire.endTransmission());
		}

		return 0xff;
	}
}