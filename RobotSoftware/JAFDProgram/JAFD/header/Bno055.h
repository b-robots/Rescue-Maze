/*
This private part of the Library is responsible for the 9 DOF-IMU (BNO055).
*/

#pragma once

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include "AllDatatypes.h"
#include "Vector.h"

namespace JAFD
{
	namespace Bno055
	{
		ReturnCode setup();
		ReturnCode calibrate();
		void updateValues();
		void tare();
		void tare(float globalHeading);		// Pitch and Roll should be 0°; Global heading in rad

		void calibToRAM();
		void calibFromRAM();

		Vec3f getLinAcc();
		Vec3f getForwardVec();
		float getRotSpeed();
	}
}
