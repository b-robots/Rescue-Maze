/*
In this file are all definitions and includes for datatypes and name aliases, which are not specialized for one module
*/

#pragma once

#include <stdint.h>
#include "Vector.h"

namespace JAFD
{
	struct FloatWheelSpeeds;

	// Speed of both wheels
	struct WheelSpeeds
	{
		int16_t left;
		int16_t right;

		WheelSpeeds(int16_t left = 0, int16_t right = 0) : left(left), right(right) {}
		WheelSpeeds(const volatile WheelSpeeds& speeds) : left(speeds.left), right(speeds.right) {}
		WheelSpeeds(const WheelSpeeds& speeds) : left(speeds.left), right(speeds.right) {}
		explicit WheelSpeeds(const volatile FloatWheelSpeeds speeds);

		inline const volatile WheelSpeeds& operator=(const volatile WheelSpeeds speeds) volatile
		{
			left = speeds.left;
			right = speeds.right;

			return *this;
		}

		inline const WheelSpeeds& operator=(const WheelSpeeds& speeds)
		{
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
		explicit FloatWheelSpeeds(const volatile WheelSpeeds speeds) : left(static_cast<float>(speeds.left)), right(static_cast<float>(speeds.right)) {}

		inline const volatile FloatWheelSpeeds& operator=(const volatile FloatWheelSpeeds speeds) volatile
		{
			left = speeds.left;
			right = speeds.right;

			return *this;
		}

		inline const FloatWheelSpeeds& operator=(const FloatWheelSpeeds& speeds)
		{
			left = speeds.left;
			right = speeds.right;

			return *this;
		}
	};

	inline WheelSpeeds::WheelSpeeds(const volatile FloatWheelSpeeds speeds) : left(static_cast<int16_t>(speeds.left)), right(static_cast<int16_t>(speeds.right)) {}

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
		Vec3f position;					// Current position (cm)

		Vec3f angularVel;				// Angular velocity as { yaw (= steering angle) / pitch (= tilt) / roll (= lean angle) } (rad/s)
		Vec3f rotation;					// Current Rotation as { yaw (= steering angle) / pitch (= tilt) / roll (= lean angle) } (rad)
	
		RobotState() : wheelSpeeds(FloatWheelSpeeds()), forwardVel(0.0f), position(Vec3f()), angularVel(Vec3f()), rotation(Vec3f()) {}
		RobotState(const volatile RobotState& state) : wheelSpeeds(state.wheelSpeeds), forwardVel(state.forwardVel), position(state.position), angularVel(state.angularVel), rotation(state.rotation) {}
		RobotState(const RobotState& state) : wheelSpeeds(state.wheelSpeeds), forwardVel(state.forwardVel), position(state.position), angularVel(state.angularVel), rotation(state.rotation) {}

		inline const volatile RobotState& operator=(const volatile RobotState state) volatile
		{
			wheelSpeeds = state.wheelSpeeds;
			forwardVel = state.forwardVel;
			position = state.position;
			angularVel = state.angularVel;
			rotation = state.rotation;

			return *this;
		}

		inline const RobotState& operator=(const RobotState& state)
		{
			wheelSpeeds = state.wheelSpeeds;
			forwardVel = state.forwardVel;
			position = state.position;
			angularVel = state.angularVel;
			rotation = state.rotation;

			return *this;
		}
	};
}