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

	// Fits angles to interval [-pi; +pi]
	inline float fitAnglesToInterval(float angle)
	{
		while (angle >= M_PI) angle -= M_TWOPI;
		while (angle <= -M_PI) angle += M_TWOPI;

		return angle;
	}

	// Fits angles to interval [-pi; +pi]
	inline Vec3f fitAnglesToInterval(Vec3f angle)
	{
		Vec3f result;

		result.x = fitAnglesToInterval(angle.x);
		result.y = fitAnglesToInterval(angle.y);
		result.z = fitAnglesToInterval(angle.z);

		return result;
	}

	// Interpolates two orientations in the correct way.
	// Angles must be in range [-180; 180]
	// if factor=0 => result=a
	// if factor=1 => result=b
	inline float interpolateAngles(float a, float b, float factor)
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

	// Interpolates two orientations in the correct way.
	// Angles must be in range [-180; 180]
	// if factor=0 => result=a
	// if factor=1 => result=b
	inline Vec3f interpolateAngles(Vec3f a, Vec3f b, float factor)
	{
		Vec3f result;

		result.x = interpolateAngles(a.x, b.x, factor);
		result.y = interpolateAngles(a.y, b.y, factor);
		result.z = interpolateAngles(a.z, b.z, factor);

		return result;
	}


	// Make a angle change coherent, based on the previous angle. Assuming the angle did not change more than 180°
	// "now" has to be in range [-pi; +pi]
	// "prev" can be anything
	inline float makeRotationCoherent(float prev, float now)
	{
		return prev + fitAnglesToInterval(now - prev);
	}

	// Make a angle change coherent, based on the previous angle. Assuming the angle did not change more than 180°
	// "now" has to be in range [-pi; +pi]
	// "prev" can be anything
	inline Vec3f makeRotationCoherent(Vec3f prev, Vec3f now)
	{
		Vec3f result;

		result.x = makeRotationCoherent(prev.x, now.x);
		result.y = makeRotationCoherent(prev.y, now.y);
		result.z = makeRotationCoherent(prev.z, now.z);

		return result;
	}
}