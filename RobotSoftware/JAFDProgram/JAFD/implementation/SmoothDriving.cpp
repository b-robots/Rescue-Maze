/*
This part of the Library is responsible for driving smoothly.
*/

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include "SmoothDriving_private.h"

namespace JAFD
{
	namespace SmoothDriving
	{
		namespace
		{
			volatile Task _currentTask;
			volatile float _currentSpeed = 0.0f;
		}
	}
}