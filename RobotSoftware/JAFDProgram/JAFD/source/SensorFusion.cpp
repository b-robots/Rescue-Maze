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
			volatile float totalHeadingOff = 0.0f;	// Total heading offset
		}

		void sensorFiltering(const uint8_t freq)
		{
			_fusedData.robotState.wheelSpeeds = MotorControl::getFloatSpeeds();
			_fusedData.robotState.angularVel = Vec3f((_fusedData.robotState.wheelSpeeds.right - _fusedData.robotState.wheelSpeeds.left) / JAFDSettings::Mechanics::wheelDistance, 0.0f, 0.0f);
			_fusedData.robotState.rotation = Vec3f((MotorControl::getDistance(Motor::right) - MotorControl::getDistance(Motor::left)) / JAFDSettings::Mechanics::wheelDistance, 0.0f, 0.0f) - Vec3f(totalHeadingOff, 0.0f, 0.0f);
			_fusedData.robotState.forwardVel = (_fusedData.robotState.wheelSpeeds.left + _fusedData.robotState.wheelSpeeds.right) / 2.0f;
			_fusedData.robotState.position += Vec3f(cosf(_fusedData.robotState.rotation.x), sinf(_fusedData.robotState.rotation.x), 0.0f) * (_fusedData.robotState.forwardVel / freq);
			_fusedData.robotState.mapCoordinate.x = roundf(_fusedData.robotState.position.x / JAFDSettings::Field::cellWidth);
			_fusedData.robotState.mapCoordinate.y = roundf(_fusedData.robotState.position.y / JAFDSettings::Field::cellWidth);
			_fusedData.robotState.mapCoordinate.floor = 0;

			float positiveAngle = _fusedData.robotState.rotation.x;

			while (positiveAngle < 0.0f) positiveAngle += M_TWOPI;
			while (positiveAngle > M_TWOPI) positiveAngle -= M_TWOPI;

			if (RAD_TO_DEG * positiveAngle > 315.0f || RAD_TO_DEG * positiveAngle < 45.0f) _fusedData.heading = AbsoluteDir::north;
			else if (RAD_TO_DEG * positiveAngle > 45.0f && RAD_TO_DEG * positiveAngle < 135.0f) _fusedData.heading = AbsoluteDir::west;
			else if (RAD_TO_DEG * positiveAngle > 135.0f && RAD_TO_DEG * positiveAngle < 225.0f) _fusedData.heading = AbsoluteDir::south;
			else _fusedData.heading = AbsoluteDir::east;
		}

		void untimedFusion()
		{
			MazeMapping::updateCurrentCell(_fusedData.gridCellCertainty, _fusedData.gridCell);
		}

		void updateDistSensor()
		{
			//_fusedData.distances.frontLong = DistanceSensors::frontLong.getDistance();
			//_fusedData.distances.backLong = DistanceSensors::backLong.getDistance();
			_fusedData.distances.frontLeft = DistanceSensors::frontLeft.getDistance();
			_fusedData.distances.frontRight = DistanceSensors::frontRight.getDistance();
			_fusedData.distances.leftBack = DistanceSensors::leftBack.getDistance();
			_fusedData.distances.leftFront = DistanceSensors::leftFront.getDistance();
			_fusedData.distances.rightBack = DistanceSensors::rightBack.getDistance();
			_fusedData.distances.rightFront = DistanceSensors::rightFront.getDistance();
		}

		void setCertainRobotPosition(Vec3f pos, Vec3f rotation)
		{
			_fusedData.robotState.rotation.y = rotation.y;
			_fusedData.robotState.rotation.z = rotation.z;

			float headingOffset = _fusedData.robotState.rotation.x - rotation.x;

			while (headingOffset < 0.0f) headingOffset += M_TWOPI;
			while (headingOffset > M_PI) headingOffset -= M_TWOPI;

			totalHeadingOff += headingOffset;

			_fusedData.robotState.rotation.x -= headingOffset;

			_fusedData.robotState.position = pos;

			_fusedData.robotState.mapCoordinate.x = roundf(_fusedData.robotState.position.x / JAFDSettings::Field::cellWidth);
			_fusedData.robotState.mapCoordinate.y = roundf(_fusedData.robotState.position.y / JAFDSettings::Field::cellWidth);
			_fusedData.robotState.mapCoordinate.floor = 0;

			if (_fusedData.robotState.rotation.x > 315.0f * DEG_TO_RAD || _fusedData.robotState.rotation.x <= 45.0f * DEG_TO_RAD) _fusedData.heading = AbsoluteDir::north;
			else if (_fusedData.robotState.rotation.x > 45.0f * DEG_TO_RAD && _fusedData.robotState.rotation.x <= 135.0f * DEG_TO_RAD) _fusedData.heading = AbsoluteDir::west;
			else if (_fusedData.robotState.rotation.x > 135.0f * DEG_TO_RAD && _fusedData.robotState.rotation.x <= 225.0f * DEG_TO_RAD) _fusedData.heading = AbsoluteDir::south;
			else _fusedData.heading = AbsoluteDir::east;
		}

		const volatile FusedData& getFusedData()
		{
			return _fusedData;
		}
	}
}