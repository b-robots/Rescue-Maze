/*
In this file are all private helper functions and private helper types (ReturnState, fastSin, fastCos...)
*/

#pragma once

#include "RobotLibraryIncludes.h"
#include "Helper_public.h"

namespace JAFTD
{
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
