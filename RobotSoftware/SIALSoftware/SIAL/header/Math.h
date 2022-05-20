#pragma once

#include "math.h"
#include "limits.h"
#include "AllDatatypes.h"

namespace SIAL
{
	inline size_t argMax(int* arr, size_t size) {
		int maxVal = INT_MIN;
		size_t maxIdx = 0;
		for (int i = 0; i < size; i++) {
			if (arr[i] >= maxVal) {
				maxVal = arr[i];
				maxIdx = i;
			}
		}

		return maxIdx;
	}

	inline void calcAngleWallOffsetFromTwoDistances(float* angle, float* distToWall, int distA, int distB, float spacing, float doubleDistToMiddle) {
		float dif = (distA - distB) / 10.0f;
		*angle = atanf(dif / spacing);
		*distToWall = (doubleDistToMiddle + (distA + distB) / 10.0f) / (2.0f * cosf(*angle));
	}

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

		if (factor > 1.0f) corrFact = 1.0f;
		else if (factor < 0.0f) corrFact = 0.0f;

		float avgSin = corrFact * sinf(b) + (1.0f - corrFact) * sinf(a);
		float avgCos = corrFact * cosf(b) + (1.0f - corrFact) * cosf(a);

		return atan2f(avgSin, avgCos);
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