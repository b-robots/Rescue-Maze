#pragma once

#include "AllDatatypes.h"
#include "Vector.h"

namespace SIAL
{
	namespace Gyro
	{
		ReturnCode setup();
		ReturnCode calibrate();
		void updateValues();
		void tare();
		void tare(float globalHeading);		// Pitch and Roll should be 0°; Global heading in rad

		void calibToRAM();
		void calibFromRAM();
		uint8_t getOverallCalibStatus();

		Vec3f getLinAcc();
		Vec3f getForwardVec();
		float getRotSpeed();
	}
}
