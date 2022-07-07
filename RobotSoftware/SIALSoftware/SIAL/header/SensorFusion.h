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
		void setUpdatedDistSens(DistSensBool distSensUpdates);
		bool scanSurrounding(uint8_t& outCumSureWalls);
		void updatePosAndRotFromDist(uint32_t time = 1000);
		float getAngleRelToWall();
		void setCorrectedState(NewForcedFusionValues newValues);
		float getAngleRelToWall();
		bool waitForAllDistSens();

		extern bool forceAnglePosReset;
		extern uint16_t consecutiveRotStuck;

		extern float distToLWall;
		extern float distToRWall;
	}
}