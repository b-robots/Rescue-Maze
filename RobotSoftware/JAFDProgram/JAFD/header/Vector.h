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
		Vec2D(T x, T y) :  x(x), y(y) {}
		Vec2D() : x(static_cast<T>(0)), y(static_cast<T>(0)) {}
	};

	template <typename T>
	class Vec3D
	{
	public:
		T x;
		T y;
		T z;
		Vec3D(T x, T y, T z) : x(x), y(y), z(z) {}
		Vec3D() : x(static_cast<T>(0)), y(static_cast<T>(0)), z(static_cast<T>(0)) {}
	};
}