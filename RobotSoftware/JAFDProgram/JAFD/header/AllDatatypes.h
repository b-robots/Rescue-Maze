/*
In this file are all definitions off all custom, global datatypes and global name aliases used in this project (excluding classes)
*/

#pragma once

#include "Vector.h"
#include <stdint.h>

namespace JAFD
{
	// Speed of both wheels
	struct WheelSpeeds
	{
		int8_t left;
		int8_t right;
	};

	// More accurate speed of both wheels
	struct FloatWheelSpeeds
	{
		float left;
		float right;
	};

	// Direction flags
	namespace Direction
	{
		constexpr uint8_t north = 1 << 0;
		constexpr uint8_t east = 1 << 1;
		constexpr uint8_t south = 1 << 2;
		constexpr uint8_t west = 1 << 3;
		constexpr uint8_t nowhere = 0;
	}

	// Which motor?
	enum class Motor : uint8_t
	{
		left,
		right
	};

	// Return codes of a function
	enum class ReturnCode : uint8_t
	{
		fatalError,
		error,
		aborted,
		ok
	};

	// State of robot
	struct RobotState
	{
		FloatWheelSpeeds wheelSpeeds;	// Speed of the wheels
		float forwardVel;				// Forward velocity (cm/s)
		float totalDistance;			// Total distance (average left and right) since last known point (cm)
		Vec3f position;					// Current position (cm)

		Vec3f angularVel;				// Angular velocity as { yaw (= steering angle) / pitch (= tilt) / roll (= lean angle) } (rad/s)
		Vec3f rotation;					// Current Rotation as { yaw (= steering angle) / pitch (= tilt) / roll (= lean angle) } (rad)
	};
}