/*
This private file of the library is responsible for math functions (fast trig functions, fast sqrt, fast invsqrt, ...)
*/

#pragma once

#include "AllDatatypes.h"

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

	// Fits angle to interval [-pi; +pi]
	inline float fitAngleToInterval(float angle)
	{
		while (angle >= M_PI) angle -= M_TWOPI;
		while (angle <= -M_PI) angle += M_TWOPI;

		return angle;
	}

	// Interpolates two orientations in the correct way.
	// Angles must be in range [-pi; pi]
	// if factor=0 => result=a
	// if factor=1 => result=b
	inline float interpolateAngle(float a, float b, float factor)
	{
		if (factor > 1.0f) factor = 1.0f;
		else if (factor < 0.0f) factor = 0.0f;

		float result;

		if (fabs(a - b) <= M_PI)
		{
			result = b * factor + a * (1.0f - factor);
		}
		else
		{
			if (a > b) b += M_TWOPI;
			else if (a < b) a += M_TWOPI;

			result = b * factor + a * (1.0f - factor);
		}

		return result;
	}

	// Make an angle change coherent, based on the previous angle. Assuming the angle did not change more than 180°
	// "now" has to be in range [-pi; +pi]
	// "prev" can be anything
	inline float makeRotationCoherent(float prev, float now)
	{
		return prev + fitAngleToInterval(now - prev);
	}


	// Get heading relative to starting position using the normalized forward vector
	inline float getGlobalHeading(Vec3f forwardVec)
	{
		fitAngleToInterval(atan2f(forwardVec.y, forwardVec.x));
	}

	// Get pitch relative to starting position using the normalized forward vector
	inline float getPitch(Vec3f forwardVec)
	{
		return asinf(forwardVec.z);
	}

	inline Vec3f toForwardVec(float globalHeading, float pitch)
	{
		Vec3f result;

		float cosPitch = cosf(pitch);

		result.z = sinf(pitch);
		result.x = cosf(globalHeading) * cosPitch;
		result.y = sinf(globalHeading) * cosPitch;
	}
}