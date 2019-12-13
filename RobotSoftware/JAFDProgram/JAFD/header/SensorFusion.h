/*
This private file of the library is responsible for the access to the SPI NVSRAM
*/

#pragma once

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include "AllDatatypes.h"
#include "Vector.h"

#include <stdint.h>

namespace JAFD
{
	namespace SensorFusion
	{
		// Current state of robot
		struct
		{
			WheelSpeeds wheelSpeeds;	// Speed of the wheels
			float forwardVel;			// Forward velocity (cm/s)
			float totalDistance;		// Total distance (average left and right) since last known point (cm)
			Vec3f position;				// Current position (cm)

			Vec3f angularVel;			// Angular velocity as { yaw (= steering angle) / pitch (= tilt) / roll (= lean angle) } (rad/s)
			Vec3f rotation;				// Current Rotation as { yaw (= steering angle) / pitch (= tilt) / roll (= lean angle) } (rad)
		} robotState;
	
		void updateSensorValues(const uint8_t freq);
	}
}