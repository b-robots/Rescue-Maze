#pragma once

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include"../header/AllDatatypes.h"

#include "U8glib.h" 

namespace JAFD
{
	namespace OLed
	{
		//Delievers an intance of the u8g Class
		U8GLIB_SSD1306_128X32* getOLedInstance();
	
		//Displays the time the Robot is startet
		void robotStopWatch(bool gamestart);

		//Displays the time is active in game
		void robotAktiveStopWatch(bool gamestart);

		//Counts down 8 Minutes
		void robotGameTimer(bool gamestart);


	}
}