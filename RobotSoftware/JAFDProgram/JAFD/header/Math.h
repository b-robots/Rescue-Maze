/*
This private file of the library is responsible for math functions (fast trig functions, fast sqrt, fast invsqrt, ...)
*/

#pragma once

#include "AllDatatypes.h"

namespace JAFD
{
	inline int8_t sgn(const int val) {
		if (val < 0) return -1;
		else if (val == 0) return 0;
		else return 1;
	}

	inline int8_t sgn(const float val) {
		if (val < 0.0f) return -1;
		else if (val == 0.0f) return 0;
		else return 1;
	}

	// Fits angle to interval [-pi; +pi]
	inline float fitAngleToInterval(const float angle)
	{
		float result = angle;

		while (result > M_PI) result -= M_TWOPI;
		while (result < -M_PI) result += M_TWOPI;

		return result;
	}

	// Interpolates two orientations in the correct way.
	// Angles must be in range [-pi; pi]
	// if factor=0 => result=a
	// if factor=1 => result=b
	inline float interpolateAngle(const float a, const float b, const float factor)
	{
		float corrFact = factor;
		float newA = a;
		float newB = b;

		if (factor > 1.0f) corrFact = 1.0f;
		else if (factor < 0.0f) corrFact = 0.0f;

		float result;

		if (fabsf(a - b) > M_PI)
		{
			if (a > b) newA += M_TWOPI;
			else newB += M_TWOPI;
		}

		result = newB * corrFact + newA * (1.0f - corrFact);
		return fitAngleToInterval(result);
	}

	// Make an angle change coherent, based on the previous angle. Assuming the angle did not change more than 180°
	// "now" has to be in range [-pi; +pi]
	// "prev" can be anything
	inline float makeRotationCoherent(const float prev, const float now)
	{
		return prev + fitAngleToInterval(now - fitAngleToInterval(prev));
	}

	// Get heading relative to starting position using the normalized forward vector
	inline float getGlobalHeading(const Vec3f& forwardVec)
	{
		return atan2f(forwardVec.y, forwardVec.x);
	}

	// Get pitch relative to starting position using the normalized forward vector
	inline float getPitch(const Vec3f& forwardVec)
	{
		return asinf(forwardVec.z);
	}

	inline Vec3f toForwardVec(const float globalHeading, const float pitch)
	{
		Vec3f result;

		float cosPitch = cosf(pitch);

		result.z = sinf(pitch);
		result.x = cosf(globalHeading) * cosPitch;
		result.y = sinf(globalHeading) * cosPitch;

		return result;
	}
}