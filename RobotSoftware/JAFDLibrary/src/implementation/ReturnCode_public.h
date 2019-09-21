/*
The return code of every function in this library
*/

#pragma once

#include <stdint.h>

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