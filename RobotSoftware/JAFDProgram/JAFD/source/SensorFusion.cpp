/*
This file of the library is responsible for the sensor fusion
*/

#pragma once

#include "../header/MazeMapping.h"
#include "../header/Math.h"
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
			volatile FusedData _fusedData;				// Fused data
			volatile float totalHeadingOff = 0.0f;		// Total heading offset
			volatile float distSensSpeed = 0.0f;		// Linear speed measured by distance sensor
			volatile bool distSensSpeedTrust = false;	// Can i trust the measured speed by the distance sensors?
		}

		void sensorFiltering(const uint8_t freq)
		{
			// Magic Numbers 1.27f and 1.05f need to be analysed ;-)
			_fusedData.robotState.wheelSpeeds = MotorControl::getFloatSpeeds();
			_fusedData.robotState.angularVel = Vec3f((_fusedData.robotState.wheelSpeeds.right - _fusedData.robotState.wheelSpeeds.left) / JAFDSettings::Mechanics::wheelDistance, 0.0f, 0.0f) / 1.27f;
			_fusedData.robotState.rotation = Vec3f((MotorControl::getDistance(Motor::right) - MotorControl::getDistance(Motor::left)) / JAFDSettings::Mechanics::wheelDistance, 0.0f, 0.0f) / 1.27f - Vec3f(totalHeadingOff, 0.0f, 0.0f);
			
			if (distSensSpeedTrust)
			{
				// !!! Faktoren sind nur num Testen
				_fusedData.robotState.forwardVel = (((_fusedData.robotState.wheelSpeeds.left + _fusedData.robotState.wheelSpeeds.right) / 2.0f / 1.05f) * 0.0f + distSensSpeed * 2.0) / 2.0f;
			}
			else
			{
				_fusedData.robotState.forwardVel = (_fusedData.robotState.wheelSpeeds.left + _fusedData.robotState.wheelSpeeds.right) / 2.0f / 1.05f;
			}

			_fusedData.robotState.position += Vec3f(cosf(_fusedData.robotState.rotation.x), sinf(_fusedData.robotState.rotation.x), 0.0f) * (_fusedData.robotState.forwardVel / freq);
			_fusedData.robotState.mapCoordinate.x = roundf(_fusedData.robotState.position.x / JAFDSettings::Field::cellWidth);
			_fusedData.robotState.mapCoordinate.y = roundf(_fusedData.robotState.position.y / JAFDSettings::Field::cellWidth);
			_fusedData.robotState.mapCoordinate.floor = 0;

			float positiveAngle = _fusedData.robotState.rotation.x;

			while (positiveAngle < 0.0f) positiveAngle += M_TWOPI;
			while (positiveAngle > M_TWOPI) positiveAngle -= M_TWOPI;

			if (RAD_TO_DEG * positiveAngle > 315.0f || RAD_TO_DEG * positiveAngle < 45.0f) _fusedData.robotState.heading = AbsoluteDir::north;
			else if (RAD_TO_DEG * positiveAngle > 45.0f && RAD_TO_DEG * positiveAngle < 135.0f) _fusedData.robotState.heading = AbsoluteDir::west;
			else if (RAD_TO_DEG * positiveAngle > 135.0f && RAD_TO_DEG * positiveAngle < 225.0f) _fusedData.robotState.heading = AbsoluteDir::south;
			else _fusedData.robotState.heading = AbsoluteDir::east;
		}

		void untimedFusion()
		{
			static uint32_t lastTime = 0;
			uint32_t now = millis();

			FusedData fusedData = _fusedData;

			// Speed measurement with distances
			uint8_t validDistSpeedSamples = 0;	// Number of valid speed measurements by distance sensor
			static uint16_t lastLeftDist = 0;	// Last distance left
			static uint16_t lastRightDist = 0;	// Last distance right
			static uint16_t lastMiddleFrontDist = 0;	// Last distance middle front
			static uint16_t lastMiddleBackDist = 0;		// Last distance middle back
			float tempDistSensSpeed = 0.0f;		// Measured speed 

			MazeMapping::updateCurrentCell(fusedData.gridCellCertainty, fusedData.gridCell);

			if (fabs(fusedData.robotState.rotation.y) < JAFDSettings::SensorFusion::maxPitchForDistSensor)
			{
				if (fusedData.distSensorState.frontLeft == DistSensorStatus::ok)
				{
					bool hitPointIsOk = false;

					// Measurement is ok
					// Check if resulting hit point is a 90° wall in front of us
					if (fusedData.robotState.heading == AbsoluteDir::north || fusedData.robotState.heading == AbsoluteDir::south)
					{
						float hitY = sinf(fusedData.robotState.rotation.x) * fusedData.distances.frontLeft / 10.0f + fusedData.robotState.position.y + sinf(fusedData.robotState.rotation.x + JAFDSettings::Mechanics::distSensFrontAngleToMiddle) * JAFDSettings::Mechanics::distSensFrontDistToMiddle;

						if (fabs(hitY - fusedData.robotState.mapCoordinate.y * JAFDSettings::Field::cellWidth) < JAFDSettings::MazeMapping::widthSecureDetectFactor * JAFDSettings::Field::cellWidth)
						{
							hitPointIsOk = true;
						}
					}
					else
					{
						float hitX = cosf(fusedData.robotState.rotation.x) * fusedData.distances.frontLeft / 10.0f + fusedData.robotState.position.x + cosf(fusedData.robotState.rotation.x + JAFDSettings::Mechanics::distSensFrontAngleToMiddle) * JAFDSettings::Mechanics::distSensFrontDistToMiddle;

						if (fabs(hitX - fusedData.robotState.mapCoordinate.x * JAFDSettings::Field::cellWidth) < JAFDSettings::MazeMapping::widthSecureDetectFactor * JAFDSettings::Field::cellWidth)
						{
							hitPointIsOk = true;
						}
					}

					if (hitPointIsOk)
					{
						if (lastLeftDist != 0 && lastTime != 0)
						{
							tempDistSensSpeed = (fusedData.distances.frontLeft - lastLeftDist) / 10.0f * 1000.0f / (now - lastTime);
							validDistSpeedSamples++;
						}

						lastLeftDist = fusedData.distances.frontLeft;
					}
					else
					{
						lastLeftDist = 0;
					}
				}
				else
				{
					lastLeftDist = 0;
				}

				if (fusedData.distSensorState.frontRight == DistSensorStatus::ok)
				{
					bool hitPointIsOk = false;

					// Measurement is ok
					// Check if resulting hit point is a 90° wall in front of us
					if (fusedData.robotState.heading == AbsoluteDir::north || fusedData.robotState.heading == AbsoluteDir::south)
					{
						float hitY = sinf(fusedData.robotState.rotation.x) * fusedData.distances.frontRight / 10.0f + fusedData.robotState.position.y - sinf(fusedData.robotState.rotation.x + JAFDSettings::Mechanics::distSensFrontAngleToMiddle) * JAFDSettings::Mechanics::distSensFrontDistToMiddle;

						if (fabs(hitY - fusedData.robotState.mapCoordinate.y * JAFDSettings::Field::cellWidth) < JAFDSettings::MazeMapping::widthSecureDetectFactor * JAFDSettings::Field::cellWidth)
						{
							hitPointIsOk = true;
						}
					}
					else
					{
						float hitX = cosf(fusedData.robotState.rotation.x) * fusedData.distances.frontRight / 10.0f + fusedData.robotState.position.x - cosf(fusedData.robotState.rotation.x + JAFDSettings::Mechanics::distSensFrontAngleToMiddle) * JAFDSettings::Mechanics::distSensFrontDistToMiddle;

						if (fabs(hitX - fusedData.robotState.mapCoordinate.x * JAFDSettings::Field::cellWidth) < JAFDSettings::MazeMapping::widthSecureDetectFactor * JAFDSettings::Field::cellWidth)
						{
							hitPointIsOk = true;
						}
					}

					if (hitPointIsOk)
					{
						if (lastRightDist != 0 && lastTime != 0)
						{
							tempDistSensSpeed += (fusedData.distances.frontRight - lastRightDist) / 10.0f * 1000.0f / (now - lastTime);
							validDistSpeedSamples++;
						}

						lastRightDist = fusedData.distances.frontRight;
					}
					else
					{
						lastRightDist = 0;
					}
				}
				else
				{
					lastRightDist = 0;
				}

				if (fusedData.distSensorState.frontLong == DistSensorStatus::ok)
				{
					bool hitPointIsOk = false;

					// Measurement is ok
					// Check if resulting hit point is a 90° wall in front of us
					if (fusedData.robotState.heading == AbsoluteDir::north || fusedData.robotState.heading == AbsoluteDir::south)
					{
						float hitY = sinf(fusedData.robotState.rotation.x) * (fusedData.distances.frontLong / 10.0f + JAFDSettings::Mechanics::distSensFrontBackDist / 2.0f) + fusedData.robotState.position.y;

						if (fabs(hitY - fusedData.robotState.mapCoordinate.y * JAFDSettings::Field::cellWidth) < JAFDSettings::MazeMapping::widthSecureDetectFactor * JAFDSettings::Field::cellWidth)
						{
							hitPointIsOk = true;
						}
					}
					else
					{
						float hitX = cosf(fusedData.robotState.rotation.x) * (fusedData.distances.frontRight / 10.0f + JAFDSettings::Mechanics::distSensFrontBackDist / 2.0f) + fusedData.robotState.position.x;

						if (fabs(hitX - fusedData.robotState.mapCoordinate.x * JAFDSettings::Field::cellWidth) < JAFDSettings::MazeMapping::widthSecureDetectFactor * JAFDSettings::Field::cellWidth)
						{
							hitPointIsOk = true;
						}
					}

					if (hitPointIsOk)
					{
						if (lastMiddleFrontDist != 0 && lastTime != 0)
						{
							Serial.println((float)lastMiddleFrontDist);
							tempDistSensSpeed += (fusedData.distances.frontLong - lastMiddleFrontDist) / 10.0f * 1000.0f / (now - lastTime);
							validDistSpeedSamples++;
						}

						lastMiddleFrontDist = fusedData.distances.frontLong;
					}
					else
					{
						lastMiddleFrontDist = 0;
					}
				}
				else
				{
					lastMiddleFrontDist = 0;
				}

				//if (fusedData.distSensorState.backLong == DistSensorStatus::ok)
				//{
				//	bool hitPointIsOk = false;

				//	// Measurement is ok
				//	// Check if resulting hit point is a 90° wall in front of us
				//	if (fusedData.robotState.heading == AbsoluteDir::north || fusedData.robotState.heading == AbsoluteDir::south)
				//	{
				//		float hitY = -sinf(fusedData.robotState.rotation.x) * (fusedData.distances.frontLong / 10.0f + JAFDSettings::Mechanics::distSensFrontBackDist / 2.0f) + fusedData.robotState.position.y;

				//		if (fabs(hitY - fusedData.robotState.mapCoordinate.y * JAFDSettings::Field::cellWidth) < JAFDSettings::MazeMapping::widthSecureDetectFactor * JAFDSettings::Field::cellWidth)
				//		{
				//			hitPointIsOk = true;
				//		}
				//	}
				//	else
				//	{
				//		float hitX = -cosf(fusedData.robotState.rotation.x) * (fusedData.distances.frontRight / 10.0f + JAFDSettings::Mechanics::distSensFrontBackDist / 2.0f) + fusedData.robotState.position.x;

				//		if (fabs(hitX - fusedData.robotState.mapCoordinate.x * JAFDSettings::Field::cellWidth) < JAFDSettings::MazeMapping::widthSecureDetectFactor * JAFDSettings::Field::cellWidth)
				//		{
				//			hitPointIsOk = true;
				//		}
				//	}

				//	if (hitPointIsOk)
				//	{
				//		if (lastMiddleBackDist != 0 && lastTime != 0)
				//		{
				//			tempDistSensSpeed -= (fusedData.distances.backLong - lastMiddleBackDist) / 10.0f * 1000.0f / (now - lastTime); // Negative, because it is the back-sensor
				//			validDistSpeedSamples++;
				//		}

				//		lastMiddleBackDist = fusedData.distances.backLong;
				//	}
				//	else
				//	{
				//		lastMiddleBackDist = 0;
				//	}
				//}
				//else
				//{
				//	lastMiddleBackDist = 0;
				//}
			}
			else
			{
				lastLeftDist = 0;
				lastRightDist = 0;
				lastMiddleFrontDist = 0;
				lastMiddleBackDist = 0;
			}

			if (validDistSpeedSamples > 0)
			{
				distSensSpeedTrust = true;
				distSensSpeed = (-tempDistSensSpeed / (float)(validDistSpeedSamples)) * JAFDSettings::SensorFusion::distSensSpeedIIRFactor + distSensSpeed * (1 - JAFDSettings::SensorFusion::distSensSpeedIIRFactor);	// Negative, because increasing distance means driving away; 
			}
			else
			{
				distSensSpeedTrust = false;
			}

			lastTime = now;

			_fusedData = fusedData;
		}

		void updateDistSensor()
		{
			uint16_t tempDist = 0.0f;
			uint32_t tempAverageDist = 0;
			uint8_t numCorrectSamples = 0;
			uint8_t numOverflowSamples = 0;
			uint8_t numUnderflowSamples = 0;

			for (uint8_t i = 0; i < JAFDSettings::DistanceSensors::averagingNumSamples; i++)
			{
				tempDist = DistanceSensors::frontLeft.getDistance();

				if (DistanceSensors::frontLeft.getStatus() == decltype(DistanceSensors::frontLeft)::Status::noError)
				{
					numCorrectSamples++;
					tempAverageDist += tempDist;
				}
				else if (DistanceSensors::frontLeft.getStatus() == decltype(DistanceSensors::frontLeft)::Status::overflow)
				{
					numOverflowSamples++;
				}
				else if (DistanceSensors::frontLeft.getStatus() == decltype(DistanceSensors::frontLeft)::Status::underflow)
				{
					numUnderflowSamples++;
				}
			}

			if (numCorrectSamples > (JAFDSettings::DistanceSensors::averagingNumSamples - numCorrectSamples))
			{
				_fusedData.distances.frontLeft = static_cast<uint16_t>(tempAverageDist / numCorrectSamples);
				_fusedData.distSensorState.frontLeft = DistSensorStatus::ok;
			}
			else
			{
				if (numOverflowSamples > (JAFDSettings::DistanceSensors::averagingNumSamples - numOverflowSamples))
				{
					_fusedData.distSensorState.frontLeft = DistSensorStatus::overflow;
				}
				else if (numUnderflowSamples > (JAFDSettings::DistanceSensors::averagingNumSamples - numUnderflowSamples))
				{
					_fusedData.distSensorState.frontLeft = DistSensorStatus::underflow;
				}
				else
				{
					_fusedData.distSensorState.frontLeft = DistSensorStatus::error;
				}

				_fusedData.distances.frontLeft = 0;
			}

			numCorrectSamples = 0;
			tempAverageDist = 0;
			numOverflowSamples = 0;
			numUnderflowSamples = 0;

			for (uint8_t i = 0; i < JAFDSettings::DistanceSensors::averagingNumSamples; i++)
			{
				tempDist = DistanceSensors::frontRight.getDistance();

				if (DistanceSensors::frontRight.getStatus() == decltype(DistanceSensors::frontRight)::Status::noError)
				{
					numCorrectSamples++;
					tempAverageDist += tempDist;
				}
				else if (DistanceSensors::frontRight.getStatus() == decltype(DistanceSensors::frontRight)::Status::overflow)
				{
					numOverflowSamples++;
				}
				else if (DistanceSensors::frontRight.getStatus() == decltype(DistanceSensors::frontRight)::Status::underflow)
				{
					numUnderflowSamples++;
				}
			}

			if (numCorrectSamples > (JAFDSettings::DistanceSensors::averagingNumSamples - numCorrectSamples))
			{
				_fusedData.distances.frontRight = static_cast<uint16_t>(tempAverageDist / numCorrectSamples);
				_fusedData.distSensorState.frontRight = DistSensorStatus::ok;
			}
			else
			{
				if (numOverflowSamples > (JAFDSettings::DistanceSensors::averagingNumSamples - numOverflowSamples))
				{
					_fusedData.distSensorState.frontRight = DistSensorStatus::overflow;
				}
				else if (numUnderflowSamples > (JAFDSettings::DistanceSensors::averagingNumSamples - numUnderflowSamples))
				{
					_fusedData.distSensorState.frontRight = DistSensorStatus::underflow;
				}
				else
				{
					_fusedData.distSensorState.frontRight = DistSensorStatus::error;
				}

				_fusedData.distances.frontRight = 0;
			}

			numCorrectSamples = 0;
			tempAverageDist = 0;
			numOverflowSamples = 0;
			numUnderflowSamples = 0;

			for (uint8_t i = 0; i < JAFDSettings::DistanceSensors::averagingNumSamples; i++)
			{
				tempDist = DistanceSensors::leftBack.getDistance();

				if (DistanceSensors::leftBack.getStatus() == decltype(DistanceSensors::leftBack)::Status::noError)
				{
					numCorrectSamples++;
					tempAverageDist += tempDist;
				}
				else if (DistanceSensors::leftBack.getStatus() == decltype(DistanceSensors::leftBack)::Status::overflow)
				{
					numOverflowSamples++;
				}
				else if (DistanceSensors::leftBack.getStatus() == decltype(DistanceSensors::leftBack)::Status::underflow)
				{
					numUnderflowSamples++;
				}
			}

			if (numCorrectSamples > (JAFDSettings::DistanceSensors::averagingNumSamples - numCorrectSamples))
			{
				_fusedData.distances.leftBack = static_cast<uint16_t>(tempAverageDist / numCorrectSamples);
				_fusedData.distSensorState.leftBack = DistSensorStatus::ok;
			}
			else
			{
				if (numOverflowSamples > (JAFDSettings::DistanceSensors::averagingNumSamples - numOverflowSamples))
				{
					_fusedData.distSensorState.leftBack = DistSensorStatus::overflow;
				}
				else if (numUnderflowSamples > (JAFDSettings::DistanceSensors::averagingNumSamples - numUnderflowSamples))
				{
					_fusedData.distSensorState.leftBack = DistSensorStatus::underflow;
				}
				else
				{
					_fusedData.distSensorState.leftBack = DistSensorStatus::error;
				}

				_fusedData.distances.leftBack = 0;
			}

			numCorrectSamples = 0;
			tempAverageDist = 0;
			numOverflowSamples = 0;
			numUnderflowSamples = 0;

			for (uint8_t i = 0; i < JAFDSettings::DistanceSensors::averagingNumSamples; i++)
			{
				tempDist = DistanceSensors::leftFront.getDistance();
				
				if (DistanceSensors::leftFront.getStatus() == decltype(DistanceSensors::leftFront)::Status::noError)
				{
					numCorrectSamples++;
					tempAverageDist += tempDist;
				}
				else if (DistanceSensors::leftFront.getStatus() == decltype(DistanceSensors::leftFront)::Status::overflow)
				{
					numOverflowSamples++;
				}
				else if (DistanceSensors::leftFront.getStatus() == decltype(DistanceSensors::leftFront)::Status::underflow)
				{
					numUnderflowSamples++;
				}
			}

			if (numCorrectSamples > (JAFDSettings::DistanceSensors::averagingNumSamples - numCorrectSamples))
			{
				_fusedData.distances.leftFront = static_cast<uint16_t>(tempAverageDist / numCorrectSamples);
				_fusedData.distSensorState.leftFront = DistSensorStatus::ok;
			}
			else
			{
				if (numOverflowSamples > (JAFDSettings::DistanceSensors::averagingNumSamples - numOverflowSamples))
				{
					_fusedData.distSensorState.leftFront = DistSensorStatus::overflow;
				}
				else if (numUnderflowSamples > (JAFDSettings::DistanceSensors::averagingNumSamples - numUnderflowSamples))
				{
					_fusedData.distSensorState.leftFront = DistSensorStatus::underflow;
				}
				else
				{
					_fusedData.distSensorState.leftFront = DistSensorStatus::error;
				}

				_fusedData.distances.leftFront = 0;
			}

			numCorrectSamples = 0;
			tempAverageDist = 0;
			numOverflowSamples = 0;
			numUnderflowSamples = 0;

			for (uint8_t i = 0; i < JAFDSettings::DistanceSensors::averagingNumSamples; i++)
			{
				tempDist = DistanceSensors::rightBack.getDistance();

				if (DistanceSensors::rightBack.getStatus() == decltype(DistanceSensors::rightBack)::Status::noError)
				{
					numCorrectSamples++;
					tempAverageDist += tempDist;
				}
				else if (DistanceSensors::rightBack.getStatus() == decltype(DistanceSensors::rightBack)::Status::overflow)
				{
					numOverflowSamples++;
				}
				else if (DistanceSensors::rightBack.getStatus() == decltype(DistanceSensors::rightBack)::Status::underflow)
				{
					numUnderflowSamples++;
				}
			}

			if (numCorrectSamples > (JAFDSettings::DistanceSensors::averagingNumSamples - numCorrectSamples))
			{
				_fusedData.distances.rightBack = static_cast<uint16_t>(tempAverageDist / numCorrectSamples);
				_fusedData.distSensorState.rightBack = DistSensorStatus::ok;
			}
			else
			{
				if (numOverflowSamples > (JAFDSettings::DistanceSensors::averagingNumSamples - numOverflowSamples))
				{
					_fusedData.distSensorState.rightBack = DistSensorStatus::overflow;
				}
				else if (numUnderflowSamples > (JAFDSettings::DistanceSensors::averagingNumSamples - numUnderflowSamples))
				{
					_fusedData.distSensorState.rightBack = DistSensorStatus::underflow;
				}
				else
				{
					_fusedData.distSensorState.rightBack = DistSensorStatus::error;
				}

				_fusedData.distances.rightBack = 0;
			}

			numCorrectSamples = 0;
			tempAverageDist = 0;
			numOverflowSamples = 0;
			numUnderflowSamples = 0;

			for (uint8_t i = 0; i < JAFDSettings::DistanceSensors::averagingNumSamples; i++)
			{
				tempDist = DistanceSensors::rightFront.getDistance();

				if (DistanceSensors::rightFront.getStatus() == decltype(DistanceSensors::rightFront)::Status::noError)
				{
					numCorrectSamples++;
					tempAverageDist += tempDist;
				}
				else if (DistanceSensors::rightFront.getStatus() == decltype(DistanceSensors::rightFront)::Status::overflow)
				{
					numOverflowSamples++;
				}
				else if (DistanceSensors::rightFront.getStatus() == decltype(DistanceSensors::rightFront)::Status::underflow)
				{
					numUnderflowSamples++;
				}
			}

			if (numCorrectSamples > (JAFDSettings::DistanceSensors::averagingNumSamples - numCorrectSamples))
			{
				_fusedData.distances.rightFront = static_cast<uint16_t>(tempAverageDist / numCorrectSamples);
				_fusedData.distSensorState.rightFront = DistSensorStatus::ok;
			}
			else
			{
				if (numOverflowSamples > (JAFDSettings::DistanceSensors::averagingNumSamples - numOverflowSamples))
				{
					_fusedData.distSensorState.rightFront = DistSensorStatus::overflow;
				}
				else if (numUnderflowSamples > (JAFDSettings::DistanceSensors::averagingNumSamples - numUnderflowSamples))
				{
					_fusedData.distSensorState.rightFront = DistSensorStatus::underflow;
				}
				else
				{
					_fusedData.distSensorState.rightFront = DistSensorStatus::error;
				}

				_fusedData.distances.rightFront = 0;
			}

			numCorrectSamples = 0;
			tempAverageDist = 0;
			numOverflowSamples = 0;
			numUnderflowSamples = 0;

			for (uint8_t i = 0; i < JAFDSettings::DistanceSensors::averagingNumSamples; i++)
			{
				tempDist = DistanceSensors::frontLong.getDistance();

				if (DistanceSensors::frontLong.getStatus() == decltype(DistanceSensors::frontLong)::Status::noError)
				{
					numCorrectSamples++;
					tempAverageDist += tempDist;
				}
				else if (DistanceSensors::frontLong.getStatus() == decltype(DistanceSensors::frontLong)::Status::overflow)
				{
					numOverflowSamples++;
				}
				else if (DistanceSensors::frontLong.getStatus() == decltype(DistanceSensors::frontLong)::Status::underflow)
				{
					numUnderflowSamples++;
				}
			}

			if (numCorrectSamples > (JAFDSettings::DistanceSensors::averagingNumSamples - numCorrectSamples))
			{
				_fusedData.distances.frontLong = static_cast<uint16_t>(tempAverageDist / numCorrectSamples);
				_fusedData.distSensorState.frontLong = DistSensorStatus::ok;
			}
			else
			{
				if (numOverflowSamples > (JAFDSettings::DistanceSensors::averagingNumSamples - numOverflowSamples))
				{
					_fusedData.distSensorState.frontLong = DistSensorStatus::overflow;
				}
				else if (numUnderflowSamples > (JAFDSettings::DistanceSensors::averagingNumSamples - numUnderflowSamples))
				{
					_fusedData.distSensorState.frontLong = DistSensorStatus::underflow;
				}
				else
				{
					_fusedData.distSensorState.frontLong = DistSensorStatus::error;
				}

				_fusedData.distances.frontLong = 0;
			}

			numCorrectSamples = 0;
			tempAverageDist = 0;
			numOverflowSamples = 0;
			numUnderflowSamples = 0;

			//for (uint8_t i = 0; i < JAFDSettings::DistanceSensors::averagingNumSamples; i++)
			//{
			//	tempDist = DistanceSensors::backLong.getDistance();

			//	if (DistanceSensors::backLong.getStatus() == decltype(DistanceSensors::backLong)::Status::noError)
			//	{
			//		numCorrectSamples++;
			//		tempAverageDist += tempDist;
			//	}
			//	else if (DistanceSensors::backLong.getStatus() == decltype(DistanceSensors::backLong)::Status::overflow)
			//	{
			//		numOverflowSamples++;
			//	}
			//	else if (DistanceSensors::backLong.getStatus() == decltype(DistanceSensors::backLong)::Status::underflow)
			//	{
			//		numUnderflowSamples++;
			//	}
			//}

			//if (numCorrectSamples > (JAFDSettings::DistanceSensors::averagingNumSamples - numCorrectSamples))
			//{
			//	_fusedData.distances.backLong = static_cast<uint16_t>(tempAverageDist / numCorrectSamples);
			//	_fusedData.distSensorState.backLong = DistSensorStatus::ok;
			//}
			//else
			//{
			//	if (numOverflowSamples > (JAFDSettings::DistanceSensors::averagingNumSamples - numOverflowSamples))
			//	{
			//		_fusedData.distSensorState.backLong = DistSensorStatus::overflow;
			//	}
			//	else if (numUnderflowSamples > (JAFDSettings::DistanceSensors::averagingNumSamples - numUnderflowSamples))
			//	{
			//		_fusedData.distSensorState.backLong = DistSensorStatus::underflow;
			//	}
			//	else
			//	{
			//		_fusedData.distSensorState.backLong = DistSensorStatus::error;
			//	}

			//	_fusedData.distances.backLong = 0;
			//}
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

			if (_fusedData.robotState.rotation.x > 315.0f * DEG_TO_RAD || _fusedData.robotState.rotation.x <= 45.0f * DEG_TO_RAD) _fusedData.robotState.heading = AbsoluteDir::north;
			else if (_fusedData.robotState.rotation.x > 45.0f * DEG_TO_RAD && _fusedData.robotState.rotation.x <= 135.0f * DEG_TO_RAD) _fusedData.robotState.heading = AbsoluteDir::west;
			else if (_fusedData.robotState.rotation.x > 135.0f * DEG_TO_RAD && _fusedData.robotState.rotation.x <= 225.0f * DEG_TO_RAD) _fusedData.robotState.heading = AbsoluteDir::south;
			else _fusedData.robotState.heading = AbsoluteDir::east;
		}

		const volatile FusedData& getFusedData()
		{
			return _fusedData;
		}
	}
}