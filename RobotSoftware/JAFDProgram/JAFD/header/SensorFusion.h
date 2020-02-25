/*
This file of the library is responsible for the sensor fusion
*/

#pragma once

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include "AllDatatypes.h"

namespace JAFD
{
	namespace SensorFusion
	{
		void sensorFiltering(const uint8_t freq);					// Apply filter and calculate robot state
		void untimedFusion();										// Update sensor values
		void updateDistSensor();									// Update distance measurements
		const volatile FusedData& getFusedData();					// Get current robot state
		void setCertainRobotPosition(Vec3f pos, Vec3f rotation);	// Set a certain robot position and angle
	}
}