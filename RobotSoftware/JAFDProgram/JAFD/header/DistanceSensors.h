#pragma once

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include "../../JAFDSettings.h"
#include "AllDatatypes.h"

namespace JAFD
{
	class A
	{
	private:
		// Alle Werte als volatile
	public:
		A(uint8_t);
		// void updateValues();
		// float getDistance();
	};

	class B
	{

	};

	namespace DistanceSensors
	{
		extern A front;
		extern B left;
	}
}