#pragma once

#include <Arduino.h>

namespace JAFD
{
	class TCA9548A
	{
	public:
		TCA9548A(uint8_t address);
		uint8_t getChannel() const;
		uint8_t selectChannel(uint8_t channel);

	private:
		static const uint8_t _maxCh = 8;
		const uint8_t _address;
		uint8_t _currentChannel;
	};
}