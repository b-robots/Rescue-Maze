/*
This private file of the library is responsible for math functions (fast trig functions, fast sqrt, fast invsqrt, ...)
*/

#pragma once

#include <stdint.h>

namespace JAFD
{
	inline int8_t sgn(int val) {
		if (val < 0) return -1;
		else if (val == 0) return 0;
		else return 1;
	}

	inline int8_t sgn(float val) {
		if (val < 0.0f) return -1;
		else return 1;
	}
}