/*
This file of the library is responsible for an template based vector class
*/

#pragma once

#include <stdint.h>

namespace JAFD
{
	template <typename T>
	class Vec2D
	{
	public:
		T x;
		T y;

	};

	template <typename T>
	class Vec3D
	{
	public:
		T x;
		T y;
		T z;
	};
}