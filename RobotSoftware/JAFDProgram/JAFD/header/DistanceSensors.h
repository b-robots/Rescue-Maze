#pragma once

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include "../../JAFDSettings.h"
#include "AllDatatypes.h"

#include <Adafruit_VL6180X.h>

namespace JAFD
{
	class VL6180
	{
	public:
		/*VL6180(uint8_t);*/
		void updateValues();
		float getDistance();
		uint8_t getStatus();
	private:
		// Alle Werte als volatile
		Adafruit_VL6180X _sensor;
		volatile float _distance;
		volatile float _surroundLight;
		volatile uint8_t _status;
	};

	class Lidar
	{

	};

	namespace DistanceSensors
	{
		extern VL6180 front;
		extern Lidar left;
	}
}