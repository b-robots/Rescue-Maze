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
		void updateSensors();										// Update all sensors
		FusedData getFusedData();				// Get current robot state
		void setDistances(Distances distances);
		void setDistSensStates(DistSensorStates distSensorStates);
		bool scanSurrounding(uint8_t &outCumSureWalls);
		void updatePosAndRotFromDist();
		float getAngleRelToWall();
		void setCorrectedState(NewForcedFusionValues newValues);
		uint32_t getConsRotStuck();

		extern volatile bool forceAnglePosReset;
	}
}