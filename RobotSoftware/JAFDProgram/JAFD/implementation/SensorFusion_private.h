/*
This private file of the library is responsible for the access to the SPI NVSRAM
*/

#pragma once

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include <stdint.h>

#include "ReturnCode_public.h"
#include "../utility/Vector_private.h"

namespace JAFD
{
	namespace SensorFusion
	{
		// Current state of robot
		struct
		{
			float forwardVel;			// Forward velocity (cm/s)
			float totalDistance;		// Total distance (average left and right) since last known point (cm)
			Vec3D<float> position;		// Current position (cm)

			Vec3D<float> angularVel;	// Angular velocity as { yaw (= steering angle) / pitch (= tilt) / roll (= lean angle) } (rad/s)
			Vec3D<float> rotation;		// Current Rotation as { yaw (= steering angle) / pitch (= tilt) / roll (= lean angle) } (rad)
		} robotState;
	}
}