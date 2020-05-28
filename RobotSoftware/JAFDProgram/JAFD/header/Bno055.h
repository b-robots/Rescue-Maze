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
		ReturnCode init();
		ReturnCode calibration();
		void update_sensorreadings();		//Funktn aus Bno055 aufrufen
		void setStartPoint();

		void Write_to_RAM();
		void Read_from_RAM();

		Vec3f get_linear_acceleration();
		Vec3f get_angular_velocity();
		Vec3f get_absolute_orientation();
		Vec3f get_gravity_vector();
	}
}
