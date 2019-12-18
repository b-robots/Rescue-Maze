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

		WheelSpeeds(int8_t left = 0.0f, int8_t right = 0.0f) : left(left), right(right) {}
		WheelSpeeds(const volatile WheelSpeeds& speeds) : left(speeds.left), right(speeds.right) {}
		WheelSpeeds(const WheelSpeeds& speeds) : left(speeds.left), right(speeds.right) {}

		inline volatile WheelSpeeds& operator=(const WheelSpeeds& speeds) volatile
		{
			if (this == &speeds)
			{
				return *this;
			}

			left = speeds.left;
			right = speeds.right;

			return *this;
		}
	};

	// More accurate speed of both wheels
	struct FloatWheelSpeeds
	{
		float left;
		float right;

		FloatWheelSpeeds(float left = 0.0f, float right = 0.0f) : left(left), right(right) {}
		FloatWheelSpeeds(const volatile FloatWheelSpeeds& speeds) : left(speeds.left), right(speeds.right) {}
		FloatWheelSpeeds(const FloatWheelSpeeds& speeds) : left(speeds.left), right(speeds.right) {}

		inline volatile FloatWheelSpeeds& operator=(const FloatWheelSpeeds& speeds) volatile
		{
			if (this == &speeds)
			{
				return *this;
			}

			left = speeds.left;
			right = speeds.right;

			return *this;
		}
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
	
		RobotState() : wheelSpeeds(FloatWheelSpeeds()), forwardVel(0.0f), totalDistance(0.0f), position(Vec3f()), angularVel(Vec3f()), rotation(Vec3f()) {}
		RobotState(const volatile RobotState& state) : wheelSpeeds(state.wheelSpeeds), forwardVel(state.forwardVel), totalDistance(state.totalDistance), position(state.position), angularVel(state.angularVel), rotation(state.rotation) {}
		RobotState(const RobotState& state) : wheelSpeeds(state.wheelSpeeds), forwardVel(state.forwardVel), totalDistance(state.totalDistance), position(state.position), angularVel(state.angularVel), rotation(state.rotation) {}

		inline volatile RobotState& operator=(const RobotState& state) volatile
		{
			if (this == &state)
			{
				return *this;
			}

			wheelSpeeds = state.wheelSpeeds;
			forwardVel = state.forwardVel;
			totalDistance = state.totalDistance;
			position = state.position;
			angularVel = state.angularVel;
			rotation = state.rotation;

			return *this;
		}
	};
}