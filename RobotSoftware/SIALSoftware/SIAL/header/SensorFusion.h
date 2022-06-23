#pragma once

#include "AllDatatypes.h"

namespace SIAL
{
	namespace SensorFusion
	{
		void sensorFusion();					// Apply filter and calculate robot state
		void distSensFusion();										// Update sensor values
		void updateSensors();										// Update all sensors
		FusedData getFusedData();									// Get current robot state
		void setDistances(Distances distances);
		void setDistSensStates(DistSensorStates distSensorStates);
		bool scanSurrounding(uint8_t& outCumSureWalls);
		void updatePosAndRotFromDist();
		float getAngleRelToWall();
		void setCorrectedState(NewForcedFusionValues newValues);

		extern bool forceAnglePosReset;
	}
}