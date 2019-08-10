/*
In this file are all helper functions and helper types (ReturnState, fastSin, fastCos...)
*/

#pragma once

#include <cstdint>

namespace JAFTD
{
	// Return codes of a function
	enum class ReturnCode : uint8_t
	{
		fatalError,
		error,
		aborted,
		ok
	};

	// Directions (can't be a enum class)
	enum Direction : uint8_t
	{
		north = 1 << 0,
		east = 1 << 1,
		south = 1 << 2,
		west = 1 << 3,
		nowhere = 0
	};
}
