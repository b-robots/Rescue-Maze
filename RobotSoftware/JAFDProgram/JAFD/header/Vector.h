/*
This file of the library is responsible for a vector class
*/

#pragma once

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include <stdint.h>
#include <cmath>

namespace JAFD
{
	class Vec3f;

	class Vec2f
	{
	public:
		float x;
		float y;

		constexpr Vec2f(float x, float y) :  x(x), y(y) {}
		constexpr Vec2f() : x(0.0f), y(0.0f) {}
		Vec2f(const volatile Vec2f& vec) : x(vec.x), y(vec.y) {}
		constexpr Vec2f(const Vec2f& vec) : x(vec.x), y(vec.y) {}
		explicit Vec2f(const volatile Vec3f& vec);
		explicit constexpr Vec2f(const Vec3f& vec);

		inline const volatile Vec2f& operator=(const volatile Vec2f vec) volatile
		{
			x = vec.x;
			y = vec.y;

			return *this;
		}

		inline const Vec2f& operator=(const Vec2f& vec)
		{
			x = vec.x;
			y = vec.y;

			return *this;
		}

		inline Vec2f operator+(const Vec2f& vec) const
		{
			return Vec2f(x + vec.x, y + vec.y);
		}

		inline Vec2f operator+(const float& val) const
		{
			return Vec2f(x + val, y + val);
		}

		inline Vec2f operator-(const Vec2f& vec) const
		{
			return Vec2f(x - vec.x, y - vec.y);
		}

		inline Vec2f operator-(const float& val) const
		{
			return Vec2f(x - val, y - val);
		}

		inline Vec2f operator*(const float& val) const
		{
			return Vec2f(x * val, y * val);
		}

		inline Vec2f operator/(const float& val) const
		{
			return Vec2f(x / val, y / val);
		}

		inline const Vec2f& operator+=(const Vec2f& vec)
		{
			*this = *this + vec;
			return *this;
		}

		inline const Vec2f& operator+=(const float& val)
		{
			*this = *this + val;
			return *this;
		}

		inline const Vec2f& operator-=(const Vec2f& vec)
		{
			*this = *this - vec;
			return *this;
		}

		inline const Vec2f& operator-=(const float& val)
		{
			*this = *this - val;
			return *this;
		}
		 
		inline const Vec2f& operator*=(const float& val)
		{
			*this = *this * val;
			return *this;
		}

		inline const Vec2f& operator/=(const float& val)
		{
			*this = *this / val;
			return *this;
		}

		inline Vec2f operator+(const volatile Vec2f vec) const volatile
		{
			return Vec2f(x + vec.x, y + vec.y);
		}

		inline Vec2f operator+(const volatile float val) const volatile
		{
			return Vec2f(x + val, y + val);
		}

		inline Vec2f operator-(const volatile Vec2f vec) const volatile
		{
			return Vec2f(x - vec.x, y - vec.y);
		}

		inline Vec2f operator-(const volatile float val) const volatile
		{
			return Vec2f(x - val, y - val);
		}

		inline Vec2f operator*(const volatile float val) const volatile
		{
			return Vec2f(x * val, y * val);
		}

		inline Vec2f operator/(const volatile float val) const volatile
		{
			return Vec2f(x / val, y / val);
		}

		inline const volatile Vec2f& operator+=(const volatile Vec2f vec) volatile
		{
			*this = *this + vec;
			return *this;
		}

		inline const volatile Vec2f& operator+=(const volatile float val) volatile
		{
			*this = *this + val;
			return *this;
		}

		inline const volatile Vec2f& operator-=(const volatile Vec2f vec) volatile
		{
			*this = *this - vec;
			return *this;
		}

		inline const volatile Vec2f& operator-=(const volatile float val) volatile
		{
			*this = *this - val;
			return *this;
		}

		inline const volatile Vec2f& operator*=(const volatile float val) volatile
		{
			*this = *this * val;
			return *this;
		}

		inline const volatile Vec2f& operator/=(const volatile float val) volatile
		{
			*this = *this / val;
			return *this;
		}

		inline float length() const volatile
		{
			return sqrtf(x * x + y * y);
		}

		inline Vec2f normalized() const volatile
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

		constexpr Vec3f(float x, float y, float z) : x(x), y(y), z(z) {}
		constexpr Vec3f() : x(0.0f), y(0.0f), z(0.0f) {}
		Vec3f(const volatile Vec3f& vec) : x(vec.x), y(vec.y), z(vec.z) {}
		constexpr Vec3f(const Vec3f& vec) : x(vec.x), y(vec.y), z(vec.z) {}
		explicit Vec3f(const volatile Vec2f& vec) : x(vec.x), y(vec.y), z(0.0f) {}
		explicit constexpr Vec3f(const Vec2f& vec) : x(vec.x), y(vec.y), z(0.0f) {}

		static Vec3f angleToDir(float yaw, float pitch)
		{
			return Vec3f(cosf(yaw) * cosf(pitch), sinf(yaw) * cosf(pitch), sinf(pitch));
		}

		inline const volatile Vec3f& operator=(const volatile Vec3f vec) volatile
		{
			x = vec.x;
			y = vec.y;
			z = vec.z;

			return *this;
		}
		
		inline const Vec3f& operator=(const Vec3f& vec)
		{
			x = vec.x;
			y = vec.y;
			z = vec.z;

			return *this;
		}

		inline Vec3f operator+(const Vec3f& vec) const
		{
			return Vec3f(x + vec.x, y + vec.y, z + vec.z);
		}

		inline Vec3f operator+(const float& val) const
		{
			return Vec3f(x + val, y + val, z + val);
		}

		inline Vec3f operator-(const Vec3f& vec) const
		{
			return Vec3f(x - vec.x, y - vec.y, z - vec.z);
		}

		inline Vec3f operator-(const float& val) const
		{
			return Vec3f(x - val, y - val, z - val);
		}

		inline Vec3f operator*(const float& val) const
		{
			return Vec3f(x * val, y * val, z * val);
		}

		inline Vec3f operator/(const float& val) const
		{
			return Vec3f(x / val, y / val, z / val);
		}

		inline const Vec3f& operator+=(const Vec3f& vec)
		{
			*this = *this + vec;
			return *this;
		}

		inline const Vec3f& operator+=(const float& val)
		{
			*this = *this + val;
			return *this;
		}

		inline const Vec3f& operator-=(const Vec3f& vec)
		{
			*this = *this - vec;
			return *this;
		}

		inline const Vec3f& operator-=(const float& val)
		{
			*this = *this - val;
			return *this;
		}

		inline const Vec3f& operator*=(const float& val)
		{
			*this = *this * val;
			return *this;
		}

		inline const Vec3f& operator/=(const float& val)
		{
			*this = *this / val;
			return *this;
		}

		inline Vec3f operator+(const volatile Vec3f vec) const volatile
		{
			return Vec3f(x + vec.x, y + vec.y, z + vec.z);
		}

		inline Vec3f operator+(const volatile float val) const volatile
		{
			return Vec3f(x + val, y + val, z + val);
		}

		inline Vec3f operator-(const volatile Vec3f vec) const volatile
		{
			return Vec3f(x - vec.x, y - vec.y, z - vec.z);
		}

		inline Vec3f operator-(const volatile float val) const volatile
		{
			return Vec3f(x - val, y - val, z - val);
		}

		inline Vec3f operator*(const volatile float val) const volatile
		{
			return Vec3f(x * val, y * val, z * val);
		}

		inline Vec3f operator/(const volatile float val) const volatile
		{
			return Vec3f(x / val, y / val, z / val);
		}

		inline const volatile Vec3f& operator+=(const volatile Vec3f vec) volatile
		{
			*this = *this + vec;
			return *this;
		}

		inline const volatile Vec3f& operator+=(const volatile float val) volatile
		{
			*this = *this + val;
			return *this;
		}

		inline const volatile Vec3f& operator-=(const volatile Vec3f vec) volatile
		{
			*this = *this - vec;
			return *this;
		}

		inline const volatile Vec3f& operator-=(const volatile float val) volatile
		{
			*this = *this - val;
			return *this;
		}

		inline const volatile Vec3f& operator*=(const volatile float val) volatile
		{
			*this = *this * val;
			return *this;
		}

		inline const volatile Vec3f& operator/=(const volatile float val) volatile
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

	inline Vec2f::Vec2f(const volatile Vec3f& vec) : x(vec.x), y(vec.y) {}
	inline constexpr Vec2f::Vec2f(const Vec3f& vec) : x(vec.x), y(vec.y) {}
}