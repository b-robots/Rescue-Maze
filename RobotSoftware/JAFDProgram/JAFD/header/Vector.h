/*
This file of the library is responsible for an template based vector class
*/

#pragma once

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

		inline Vec2f& operator+=(const Vec2f& vec)
		{
			*this = *this + vec;
			return *this;
		}

		inline Vec2f& operator+=(const float& val)
		{
			*this = *this + val;
			return *this;
		}

		inline Vec2f& operator-=(const Vec2f& vec)
		{
			*this = *this - vec;
			return *this;
		}

		inline Vec2f& operator-=(const float& val)
		{
			*this = *this - val;
			return *this;
		}

		inline Vec2f& operator*=(const float& val)
		{
			*this = *this * val;
			return *this;
		}

		inline Vec2f& operator/=(const float& val)
		{
			*this = *this / val;
			return *this;
		}

		inline float length()
		{
			return sqrtf(x * x + y * y);
		}

		inline Vec2f normalized()
		{
			return *this / length();
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
			return Vec3f(x - vec.x, y - vec.y, z -vec.z);
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
			return Vec3f(x / val, y / val, z * val);
		}

		inline Vec3f& operator+=(const Vec3f& vec)
		{
			*this = *this + vec;
			return *this;
		}

		inline Vec3f& operator+=(const float& val)
		{
			*this = *this + val;
			return *this;
		}

		inline Vec3f& operator-=(const Vec3f& vec)
		{
			*this = *this - vec;
			return *this;
		}

		inline Vec3f& operator-=(const float& val)
		{
			*this = *this - val;
			return *this;
		}

		inline Vec3f& operator*=(const float& val)
		{
			*this = *this * val;
			return *this;
		}

		inline Vec3f& operator/=(const float& val)
		{
			*this = *this / val;
			return *this;
		}

		inline float length() const
		{
			return sqrtf(x * x + y * y + z * z);
		}

		inline Vec3f normalized()
		{
			return *this / length();
		}
	};

	inline Vec2f::Vec2f(const Vec3f& vec) : x(vec.x), y(vec.y) {}
}