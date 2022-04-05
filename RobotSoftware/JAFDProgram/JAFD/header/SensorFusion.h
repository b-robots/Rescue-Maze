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
		void setDrivingStraight(bool _isDrivingStraight);
		void sensorFiltering(const uint8_t freq);					// Apply filter and calculate robot state
		void untimedFusion();										// Update sensor values
		void updateSensors();										// Update all sensors
		FusedData getFusedData();									// Get current robot state
		void setCertainRobotPosition(Vec3f pos, float heading);		// Set a certain robot position and angle
		void setDistances(Distances distances);
		void setDistSensStates(DistSensorStates distSensorStates);
		bool scanSurrounding();
		void updatePosAngleFromDistSens();
		void setMapPosHeading(MapCoordinate coor, AbsoluteDir heading);

		extern volatile bool bnoErr;
	}
}