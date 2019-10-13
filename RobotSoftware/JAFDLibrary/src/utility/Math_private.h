/*
This private file of the library is responsible for math functions (fast trig functions, fast sqrt, fast invsqrt, ...)
*/

#pragma once

#include <stdint.h>

// Constexpr version of log2

namespace JAFD
{
	namespace Math
	{
		constexpr uint8_t log2(uint32_t n)
		{
			return ((n < 2) ? 1 : 1 + log2(n / 2));
		}
	}
}