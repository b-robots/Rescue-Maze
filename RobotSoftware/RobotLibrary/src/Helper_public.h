/*
In this file are all public helper functions and public helper types (ReturnState, fastSin, fastCos...)
*/

#pragma once

#include "RobotLibraryIncludes.h"

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
}
