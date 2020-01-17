/*
This private part of the Library is responsible for the 9 DOF-IMU (BNO055).
*/

#pragma once
#include "Vector.h"


namespace JAFD
{
	namespace Bno055
	{
		void init();
		void calibration();
		void update_sensorreadings();		//Funktn aus Bno055 aufrufen

		Vec3f get_linear_acceleration();
		Vec3f get_angular_velocity();
		Vec3f get_absolute_orientation();
		Vec3f get_gravity_vector();


	
	
		


	}
}
