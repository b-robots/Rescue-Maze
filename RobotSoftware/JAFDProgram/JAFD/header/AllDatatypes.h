/*
In this file are all definitions off all custom, global datatypes and global name aliases used in this project (excluding classes)
*/

#pragma once

#include <stdint.h>

namespace JAFD
{
	// Speed of both wheels
	struct WheelSpeeds
	{
		uint8_t left;
		uint8_t right;
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
}