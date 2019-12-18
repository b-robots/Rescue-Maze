/*
This file of the library is responsible for an template based vector class
*/

#pragma once

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include <stdint.h>

namespace JAFD
{
	class Vec3f;

	class Vec2f
	{
	public:
		float x;
		float y;

		Vec2f(float x, float y) :  x(x), y(y) {}
		Vec2f() : x(0.0f), y(0.0f) {}
		explicit Vec2f(const Vec3f& vec);

		inline volatile Vec2f& operator=(const Vec2f& vec) volatile
		{
			if (this == &vec)
			{
				return *this;
			}

			x = vec.x;
			y = vec.y;

			return *this;
		}

		inline Vec2f operator+(const Vec2f& vec) const volatile
		{
			return Vec2f(x + vec.x, y + vec.y);
		}

		inline Vec2f operator+(const float& val) const volatile
		{
			return Vec2f(x + val, y + val);
		}

		inline Vec2f operator-(const Vec2f& vec) const volatile
		{
			return Vec2f(x - vec.x, y - vec.y);
		}

		inline Vec2f operator-(const float& val) const volatile
		{
			return Vec2f(x - val, y - val);
		}

		inline Vec2f operator*(const float& val) const volatile
		{
			return Vec2f(x * val, y * val);
		}

		inline Vec2f operator/(const float& val) const volatile
		{
			return Vec2f(x / val, y / val);
		}

		inline volatile Vec2f& operator+=(const Vec2f& vec) volatile
		{
			*this = *this + vec;
			return *this;
		}

		inline volatile Vec2f& operator+=(const float& val) volatile
		{
			*this = *this + val;
			return *this;
		}

		inline volatile Vec2f& operator-=(const Vec2f& vec) volatile
		{
			*this = *this - vec;
			return *this;
		}

		inline volatile Vec2f& operator-=(const float& val) volatile
		{
			*this = *this - val;
			return *this;
		}
		 
		inline volatile Vec2f& operator*=(const float& val) volatile
		{
			*this = *this * val;
			return *this;
		}

		inline volatile Vec2f& operator/=(const float& val) volatile
		{
			*this = *this / val;
			return *this;
		}

		inline Vec2f operator+(const volatile Vec2f& vec) const volatile
		{
			return Vec2f(x + vec.x, y + vec.y);
		}

		inline Vec2f operator+(const volatile float& val) const volatile
		{
			return Vec2f(x + val, y + val);
		}

		inline Vec2f operator-(const volatile Vec2f& vec) const volatile
		{
			return Vec2f(x - vec.x, y - vec.y);
		}

		inline Vec2f operator-(const volatile float& val) const volatile
		{
			return Vec2f(x - val, y - val);
		}

		inline Vec2f operator*(const volatile float& val) const volatile
		{
			return Vec2f(x * val, y * val);
		}

		inline Vec2f operator/(const volatile float& val) const volatile
		{
			return Vec2f(x / val, y / val);
		}

		inline volatile Vec2f& operator+=(const volatile Vec2f& vec) volatile
		{
			*this = *this + vec;
			return *this;
		}

		inline volatile Vec2f& operator+=(const volatile float& val) volatile
		{
			*this = *this + val;
			return *this;
		}

		inline volatile Vec2f& operator-=(const volatile Vec2f& vec) volatile
		{
			*this = *this - vec;
			return *this;
		}

		inline volatile Vec2f& operator-=(const volatile float& val) volatile
		{
			*this = *this - val;
			return *this;
		}

		inline volatile Vec2f& operator*=(const volatile float& val) volatile
		{
			*this = *this * val;
			return *this;
		}

		inline volatile Vec2f& operator/=(const volatile float& val) volatile
		{
			*this = *this / val;
			return *this;
		}

		inline float length() volatile
		{
			return sqrtf(x * x + y * y);
		}

		inline Vec2f normalized() volatile
		{
			return Vec2f(x / length(), y / length());
		}
	};

	class Vec3f
	{
	public:
		float x;
		float y;
		float z;

		Vec3f(float x, float y, float z) : x(x), y(y), z(z) {}
		Vec3f() : x(0.0f), y(0.0f), z(0.0f) {}
		explicit Vec3f(const Vec2f& vec) : x(vec.x), y(vec.y), z(0.0f) {}

		inline volatile Vec3f& operator=(const Vec3f& vec) volatile
		{
			if (this == &vec)
			{
				return *this;
			}

			x = vec.x;
			y = vec.y;

			return *this;
		}

		inline Vec3f operator+(const Vec3f& vec) const volatile
		{
			return Vec3f(x + vec.x, y + vec.y, z + vec.z);
		}

		inline Vec3f operator+(const float& val) const volatile
		{
			return Vec3f(x + val, y + val, z + val);
		}

		inline Vec3f operator-(const Vec3f& vec) const volatile
		{
			return Vec3f(x - vec.x, y - vec.y, z - vec.z);
		}

		inline Vec3f operator-(const float& val) const volatile
		{
			return Vec3f(x - val, y - val, z - val);
		}

		inline Vec3f operator*(const float& val) const volatile
		{
			return Vec3f(x * val, y * val, z * val);
		}

		inline Vec3f operator/(const float& val) const volatile
		{
			return Vec3f(x / val, y / val, z / val);
		}

		inline volatile Vec3f& operator+=(const Vec3f& vec) volatile
		{
			*this = *this + vec;
			return *this;
		}

		inline volatile Vec3f& operator+=(const float& val) volatile
		{
			*this = *this + val;
			return *this;
		}

		inline volatile Vec3f& operator-=(const Vec3f& vec) volatile
		{
			*this = *this - vec;
			return *this;
		}

		inline volatile Vec3f& operator-=(const float& val) volatile
		{
			*this = *this - val;
			return *this;
		}

		inline volatile Vec3f& operator*=(const float& val) volatile
		{
			*this = *this * val;
			return *this;
		}

		inline volatile Vec3f& operator/=(const float& val) volatile
		{
			*this = *this / val;
			return *this;
		}

		inline Vec3f operator+(const volatile Vec3f& vec) const volatile
		{
			return Vec3f(x + vec.x, y + vec.y, z + vec.z);
		}

		inline Vec3f operator+(const volatile float& val) const volatile
		{
			return Vec3f(x + val, y + val, z + val);
		}

		inline Vec3f operator-(const volatile Vec3f& vec) const volatile
		{
			return Vec3f(x - vec.x, y - vec.y, z - vec.z);
		}

		inline Vec3f operator-(const volatile float& val) const volatile
		{
			return Vec3f(x - val, y - val, z - val);
		}

		inline Vec3f operator*(const volatile float& val) const volatile
		{
			return Vec3f(x * val, y * val, z * val);
		}

		inline Vec3f operator/(const volatile float& val) const volatile
		{
			return Vec3f(x / val, y / val, z / val);
		}

		inline volatile Vec3f& operator+=(const volatile Vec3f& vec) volatile
		{
			*this = *this + vec;
			return *this;
		}

		inline volatile Vec3f& operator+=(const volatile float& val) volatile
		{
			*this = *this + val;
			return *this;
		}

		inline volatile Vec3f& operator-=(const volatile Vec3f& vec) volatile
		{
			*this = *this - vec;
			return *this;
		}

		inline volatile Vec3f& operator-=(const volatile float& val) volatile
		{
			*this = *this - val;
			return *this;
		}

		inline volatile Vec3f& operator*=(const volatile float& val) volatile
		{
			*this = *this * val;
			return *this;
		}

		inline volatile Vec3f& operator/=(const volatile float& val) volatile
		{
			*this = *this / val;
			return *this;
		}

		inline float length() const volatile
		{
			return sqrtf(x * x + y * y);
		}

		inline Vec3f normalized() const volatile
		{
			return Vec3f(x / length(), y / length(), z / length());
		}
	};

	inline Vec2f::Vec2f(const Vec3f& vec) : x(vec.x), y(vec.y) {}
}