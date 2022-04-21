/*
This private part of the Library is responsible for the communication with the RasPI for the camera recognition.
*/

#pragma once

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include "AllDatatypes.h"

namespace JAFD
{
	namespace CamRec
	{
		ReturnCode setup();

		void loop();

		Victim getVictim(bool left);
	}
}