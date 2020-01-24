/*
This file of the library is responsible for the sensor fusion
*/

#pragma once

#include "../header/MazeMapping.h"
#include "../header/SensorFusion.h"
#include "../header/MotorControl.h"
#include "../header/DistanceSensors.h"
#include "../../JAFDSettings.h"
#include <math.h>

namespace JAFD
{
	namespace SensorFusion
	{
		namespace
		{
			volatile FusedData _fusedData;	// Fused data
		}

		void timedSensorUpdate(const uint8_t freq)
		{
			_fusedData.robotState.angularVel = Vec3f((_fusedData.robotState.wheelSpeeds.right - _fusedData.robotState.wheelSpeeds.left) / JAFDSettings::Mechanics::wheelDistance, 0.0f, 0.0f);
			_fusedData.robotState.rotation += _fusedData.robotState.angularVel / freq;
			_fusedData.robotState.forwardVel = (_fusedData.robotState.wheelSpeeds.left + _fusedData.robotState.wheelSpeeds.right) / 2.0f;
			_fusedData.robotState.position += Vec3f(cosf(_fusedData.robotState.rotation.x), sinf(_fusedData.robotState.rotation.x), 0.0f) * (_fusedData.robotState.forwardVel / freq);
			_fusedData.robotState.mapCoordinate.x = _fusedData.robotState.position.x * 2.0f / JAFDSettings::Field::cellWidth;
			_fusedData.robotState.mapCoordinate.y = _fusedData.robotState.position.y * 2.0f / JAFDSettings::Field::cellWidth;
			_fusedData.robotState.mapCoordinate.floor = 0;

			float positiveAngle = _fusedData.robotState.rotation.x;

			while (positiveAngle < 0.0f) positiveAngle += 360.0f;
			while (positiveAngle > 360.0f) positiveAngle -= 360.0f;

			if (_fusedData.robotState.rotation.x > 315.0f || _fusedData.robotState.rotation.x < 45.0f) _fusedData.heading = HeadingDirection::north;
			else if (_fusedData.robotState.rotation.x > 45.0f && _fusedData.robotState.rotation.x < 135.0f) _fusedData.heading = HeadingDirection::east;
			else if (_fusedData.robotState.rotation.x > 135.0f && _fusedData.robotState.rotation.x < 225.0f) _fusedData.heading = HeadingDirection::south;
			else _fusedData.heading = HeadingDirection::west;
		}

		void untimedSensorUpdate()
		{
			_fusedData.robotState.wheelSpeeds = MotorControl::getFloatSpeeds();
			
			DistanceSensors::frontLeft.updateValues();
			DistanceSensors::frontRight.updateValues();
			DistanceSensors::frontLong.updateValues();
			DistanceSensors::backLong.updateValues();
			DistanceSensors::leftFront.updateValues();
			DistanceSensors::leftBack.updateValues();
			DistanceSensors::rightFront.updateValues();
			DistanceSensors::rightBack.updateValues();

			MazeMapping::updateCurrentCell(_fusedData.gridCellCertainty, _fusedData.gridCell);
		}

		const volatile FusedData& getFusedData()
		{
			return _fusedData;
		}
	}
}