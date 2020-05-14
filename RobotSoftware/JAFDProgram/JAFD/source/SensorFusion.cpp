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
			volatile FusedData fusedData;				// Fused data
			volatile float totalHeadingOff = 0.0f;		// Total heading offset
			volatile float distSensSpeed = 0.0f;		// Linear speed measured by distance sensors
			volatile float distSensSpeedTrust = 0.0f;	// How much can I trust the measured speed by the distance sensors? (0.0 - 1.0)
			volatile float distSensAngle = 0.0f;		// Angle measured by distance sensors
			volatile float distSensAngleTrust = 0.0f;	// How much can I trust the measured angle? (0.0 - 1.0)
		}

		void sensorFiltering(const uint8_t freq)
		{
			// Magic Numbers 1.27f and 1.05f need to be analysed ;-)
			fusedData.robotState.wheelSpeeds = MotorControl::getFloatSpeeds();
			fusedData.robotState.angularVel = Vec3f((fusedData.robotState.wheelSpeeds.right - fusedData.robotState.wheelSpeeds.left) / JAFDSettings::Mechanics::wheelDistance, 0.0f, 0.0f) / 1.27f;
			fusedData.robotState.rotation = Vec3f((MotorControl::getDistance(Motor::right) - MotorControl::getDistance(Motor::left)) / JAFDSettings::Mechanics::wheelDistance, 0.0f, 0.0f) / 1.27f - Vec3f(totalHeadingOff, 0.0f, 0.0f);

			fusedData.robotState.rotation.x = DEG_TO_RAD * 460.0f;

			// Nut zum testen
			if (distSensSpeedTrust > 0.0f)
			{
				// !!! Faktoren sind nur num Testen
				fusedData.robotState.forwardVel = (((fusedData.robotState.wheelSpeeds.left + fusedData.robotState.wheelSpeeds.right) / 2.0f / 1.05f) * 0.0f + distSensSpeed * 2.0) / 2.0f;
			}
			else
			{
				fusedData.robotState.forwardVel = (fusedData.robotState.wheelSpeeds.left + fusedData.robotState.wheelSpeeds.right) / 2.0f / 1.05f;
			}

			fusedData.robotState.position += Vec3f(cosf(fusedData.robotState.rotation.x), sinf(fusedData.robotState.rotation.x), 0.0f) * (fusedData.robotState.forwardVel / freq);
			fusedData.robotState.mapCoordinate.x = roundf(fusedData.robotState.position.x / JAFDSettings::Field::cellWidth);
			fusedData.robotState.mapCoordinate.y = roundf(fusedData.robotState.position.y / JAFDSettings::Field::cellWidth);
			fusedData.robotState.mapCoordinate.floor = 0;

			float positiveAngle = fusedData.robotState.rotation.x;

			while (positiveAngle < 0.0f) positiveAngle += M_TWOPI;
			while (positiveAngle > M_TWOPI) positiveAngle -= M_TWOPI;

			if (RAD_TO_DEG * positiveAngle > 315.0f || RAD_TO_DEG * positiveAngle < 45.0f) fusedData.robotState.heading = AbsoluteDir::north;
			else if (RAD_TO_DEG * positiveAngle > 45.0f && RAD_TO_DEG * positiveAngle < 135.0f) fusedData.robotState.heading = AbsoluteDir::west;
			else if (RAD_TO_DEG * positiveAngle > 135.0f && RAD_TO_DEG * positiveAngle < 225.0f) fusedData.robotState.heading = AbsoluteDir::south;
			else fusedData.robotState.heading = AbsoluteDir::east;
		}

		void untimedFusion()
		{
			static uint32_t lastTime = 0;
			uint32_t now = millis();

			FusedData tempFusedData = fusedData;

			// Speed measurement with distances
			uint8_t validDistSpeedSamples = 0;			// Number of valid speed measurements by distance sensor
			static uint16_t lastLeftDist = 0;			// Last distance left
			static uint16_t lastRightDist = 0;			// Last distance right
			static uint16_t lastMiddleFrontDist = 0;	// Last distance middle front
			static uint16_t lastMiddleBackDist = 0;		// Last distance middle back
			float tempDistSensSpeed = 0.0f;				// Measured speed 

			float tempDistSensAngle = 0.0f;				// Angle measured by distance sensors
			float tempDistSensAngleTrust = 0.0f;		// How much can I trust the measured angle? (0.0 - 1.0)

			uint8_t frontWallsDetected = 0;		// How many times did a wall in front of us get detected
			uint8_t leftWallsDetected = 0;		// How many times did a wall left of us get detected
			uint8_t rightWallsDetected = 0;		// How many times did a wall right of us get detected

			if (fabs(tempFusedData.robotState.rotation.y) < JAFDSettings::SensorFusion::maxPitchForDistSensor)
			{
				if (tempFusedData.distSensorState.frontLeft == DistSensorStatus::ok)
				{
					bool hitPointIsOk = false;

					// Measurement is ok
					// Check if resulting hit point is a 90° wall in front of us
					if (tempFusedData.robotState.heading == AbsoluteDir::north || tempFusedData.robotState.heading == AbsoluteDir::south)
					{
						float hitY = sinf(tempFusedData.robotState.rotation.x) * tempFusedData.distances.frontLeft / 10.0f + tempFusedData.robotState.position.y + sinf(tempFusedData.robotState.rotation.x + JAFDSettings::Mechanics::distSensFrontAngleToMiddle) * JAFDSettings::Mechanics::distSensFrontDistToMiddle;

						if (fabs(hitY - tempFusedData.robotState.mapCoordinate.y * JAFDSettings::Field::cellWidth) < JAFDSettings::MazeMapping::widthSecureDetectFactor * JAFDSettings::Field::cellWidth / 2.0f)
						{
							hitPointIsOk = true;

							float hitX = cosf(tempFusedData.robotState.rotation.x) * tempFusedData.distances.frontLeft / 10.0f + tempFusedData.robotState.position.x + cosf(tempFusedData.robotState.rotation.x + JAFDSettings::Mechanics::distSensFrontAngleToMiddle) * JAFDSettings::Mechanics::distSensFrontDistToMiddle;

							if (fabs(hitX - tempFusedData.robotState.mapCoordinate.x * JAFDSettings::Field::cellWidth) < JAFDSettings::Field::cellWidth / 2.0f + JAFDSettings::MazeMapping::distLongerThanBorder)
							{
								// Wall is directly in front of us
								frontWallsDetected++;
							}
						}
					}
					else
					{
						float hitX = cosf(tempFusedData.robotState.rotation.x) * tempFusedData.distances.frontLeft / 10.0f + tempFusedData.robotState.position.x + cosf(tempFusedData.robotState.rotation.x + JAFDSettings::Mechanics::distSensFrontAngleToMiddle) * JAFDSettings::Mechanics::distSensFrontDistToMiddle;

						if (fabs(hitX - tempFusedData.robotState.mapCoordinate.x * JAFDSettings::Field::cellWidth) < JAFDSettings::MazeMapping::widthSecureDetectFactor * JAFDSettings::Field::cellWidth / 2.0f)
						{
							hitPointIsOk = true;

							float hitY = sinf(tempFusedData.robotState.rotation.x) * tempFusedData.distances.frontLeft / 10.0f + tempFusedData.robotState.position.y + sinf(tempFusedData.robotState.rotation.x + JAFDSettings::Mechanics::distSensFrontAngleToMiddle) * JAFDSettings::Mechanics::distSensFrontDistToMiddle;

							if (fabs(hitY - tempFusedData.robotState.mapCoordinate.y * JAFDSettings::Field::cellWidth) < JAFDSettings::Field::cellWidth / 2.0f + JAFDSettings::MazeMapping::distLongerThanBorder)
							{
								// Wall is directly in front of us
								frontWallsDetected++;
							}
						}
					}

					if (hitPointIsOk)
					{
						if (lastLeftDist != 0 && lastTime != 0)
						{
							tempDistSensSpeed = (tempFusedData.distances.frontLeft - lastLeftDist) / 10.0f * 1000.0f / (now - lastTime);
							validDistSpeedSamples++;
						}

						lastLeftDist = tempFusedData.distances.frontLeft;
					}
					else
					{
						lastLeftDist = 0;
					}
				}
				else
				{
					lastLeftDist = 0;

					if (tempFusedData.distSensorState.frontLeft == DistSensorStatus::underflow)
					{
						frontWallsDetected++;
					}
				}

				if (tempFusedData.distSensorState.frontRight == DistSensorStatus::ok)
				{
					bool hitPointIsOk = false;

					// Measurement is ok
					// Check if resulting hit point is a 90° wall in front of us
					if (tempFusedData.robotState.heading == AbsoluteDir::north || tempFusedData.robotState.heading == AbsoluteDir::south)
					{
						float hitY = sinf(tempFusedData.robotState.rotation.x) * tempFusedData.distances.frontRight / 10.0f + tempFusedData.robotState.position.y + sinf(tempFusedData.robotState.rotation.x - JAFDSettings::Mechanics::distSensFrontAngleToMiddle) * JAFDSettings::Mechanics::distSensFrontDistToMiddle;

						if (fabs(hitY - tempFusedData.robotState.mapCoordinate.y * JAFDSettings::Field::cellWidth) < JAFDSettings::MazeMapping::widthSecureDetectFactor * JAFDSettings::Field::cellWidth / 2.0f)
						{
							hitPointIsOk = true;

							float hitX = cosf(tempFusedData.robotState.rotation.x) * tempFusedData.distances.frontRight / 10.0f + tempFusedData.robotState.position.x + cosf(tempFusedData.robotState.rotation.x - JAFDSettings::Mechanics::distSensFrontAngleToMiddle) * JAFDSettings::Mechanics::distSensFrontDistToMiddle;

							if (fabs(hitX - tempFusedData.robotState.mapCoordinate.x * JAFDSettings::Field::cellWidth) < JAFDSettings::Field::cellWidth / 2.0f + JAFDSettings::MazeMapping::distLongerThanBorder)
							{
								// Wall is directly in front of us
								frontWallsDetected++;
							}
						}
					}
					else
					{
						float hitX = cosf(tempFusedData.robotState.rotation.x) * tempFusedData.distances.frontRight / 10.0f + tempFusedData.robotState.position.x + cosf(tempFusedData.robotState.rotation.x - JAFDSettings::Mechanics::distSensFrontAngleToMiddle) * JAFDSettings::Mechanics::distSensFrontDistToMiddle;

						if (fabs(hitX - tempFusedData.robotState.mapCoordinate.x * JAFDSettings::Field::cellWidth) < JAFDSettings::MazeMapping::widthSecureDetectFactor * JAFDSettings::Field::cellWidth / 2.0f)
						{
							hitPointIsOk = true;

							float hitY = sinf(tempFusedData.robotState.rotation.x) * tempFusedData.distances.frontRight / 10.0f + tempFusedData.robotState.position.y + sinf(tempFusedData.robotState.rotation.x - JAFDSettings::Mechanics::distSensFrontAngleToMiddle) * JAFDSettings::Mechanics::distSensFrontDistToMiddle;

							if (fabs(hitY - tempFusedData.robotState.mapCoordinate.y * JAFDSettings::Field::cellWidth) < JAFDSettings::Field::cellWidth / 2.0f + JAFDSettings::MazeMapping::distLongerThanBorder)
							{
								// Wall is directly in front of us
								frontWallsDetected++;
							}
						}
					}

					if (hitPointIsOk)
					{
						if (lastRightDist != 0 && lastTime != 0)
						{
							tempDistSensSpeed += (tempFusedData.distances.frontRight - lastRightDist) / 10.0f * 1000.0f / (now - lastTime);
							validDistSpeedSamples++;
						}

						lastRightDist = tempFusedData.distances.frontRight;
					}
					else
					{
						lastRightDist = 0;
					}
				}
				else
				{
					lastRightDist = 0;

					if (tempFusedData.distSensorState.frontRight == DistSensorStatus::underflow)
					{
						frontWallsDetected++;
					}
				}

				if (frontWallsDetected == 2 && tempFusedData.distSensorState.frontLeft == DistSensorStatus::ok && tempFusedData.distSensorState.frontRight == DistSensorStatus::ok)
				{
					// Calculate angle if both front distance sensors detected a wall directly in front of the robot.
					tempDistSensAngle += asinf((tempFusedData.distances.frontLeft - tempFusedData.distances.frontRight) / sqrtf(JAFDSettings::Mechanics::distSensFrontSpacing * JAFDSettings::Mechanics::distSensFrontSpacing * 100.0f + (tempFusedData.distances.frontLeft - tempFusedData.distances.frontRight) * (tempFusedData.distances.frontLeft - tempFusedData.distances.frontRight)));
					tempDistSensAngleTrust += 1.0f / 3.0f;
				}

				if (tempFusedData.distSensorState.leftFront == DistSensorStatus::ok)
				{
					// Measurement is ok
					// Check if resulting hit point is a 90° wall in front of us
					if (tempFusedData.robotState.heading == AbsoluteDir::north || tempFusedData.robotState.heading == AbsoluteDir::south)
					{
						float hitX = -sinf(tempFusedData.robotState.rotation.x) * tempFusedData.distances.leftFront / 10.0f + tempFusedData.robotState.position.x - sinf(tempFusedData.robotState.rotation.x - JAFDSettings::Mechanics::distSensLRAngleToMiddle) * JAFDSettings::Mechanics::distSensLRDistToMiddle;

						if (fabs(hitX - tempFusedData.robotState.mapCoordinate.x * JAFDSettings::Field::cellWidth) < JAFDSettings::MazeMapping::widthSecureDetectFactor * JAFDSettings::Field::cellWidth / 2.0f)
						{
							float hitY = cosf(tempFusedData.robotState.rotation.x) * tempFusedData.distances.leftFront / 10.0f + tempFusedData.robotState.position.y + cosf(tempFusedData.robotState.rotation.x - JAFDSettings::Mechanics::distSensLRAngleToMiddle) * JAFDSettings::Mechanics::distSensLRDistToMiddle;

							if (fabs(hitY - tempFusedData.robotState.mapCoordinate.y * JAFDSettings::Field::cellWidth) < JAFDSettings::Field::cellWidth / 2.0f + JAFDSettings::MazeMapping::distLongerThanBorder)
							{
								// Wall is directly left of us
								leftWallsDetected++;
							}
						}
					}
					else
					{
						float hitY = cosf(tempFusedData.robotState.rotation.x) * tempFusedData.distances.leftFront / 10.0f + tempFusedData.robotState.position.y + cosf(tempFusedData.robotState.rotation.x + JAFDSettings::Mechanics::distSensLRAngleToMiddle) * JAFDSettings::Mechanics::distSensLRDistToMiddle;

						if (fabs(hitY - tempFusedData.robotState.mapCoordinate.y * JAFDSettings::Field::cellWidth) < JAFDSettings::MazeMapping::widthSecureDetectFactor * JAFDSettings::Field::cellWidth / 2.0f)
						{
							float hitX = -sinf(tempFusedData.robotState.rotation.x) * tempFusedData.distances.leftFront / 10.0f + tempFusedData.robotState.position.x - sinf(tempFusedData.robotState.rotation.x + JAFDSettings::Mechanics::distSensLRAngleToMiddle) * JAFDSettings::Mechanics::distSensLRDistToMiddle;

							if (fabs(hitX - tempFusedData.robotState.mapCoordinate.x * JAFDSettings::Field::cellWidth) < JAFDSettings::Field::cellWidth / 2.0f + JAFDSettings::MazeMapping::distLongerThanBorder)
							{
								// Wall is directly left of us
								leftWallsDetected++;
							}
						}
					}		
				}
				else if (tempFusedData.distSensorState.leftFront == DistSensorStatus::underflow)
				{
					leftWallsDetected++;
				}

				if (tempFusedData.distSensorState.leftBack == DistSensorStatus::ok)
				{
					// Measurement is ok
					// Check if resulting hit point is a 90° wall in front of us
					if (tempFusedData.robotState.heading == AbsoluteDir::north || tempFusedData.robotState.heading == AbsoluteDir::south)
					{
						float hitX = -sinf(tempFusedData.robotState.rotation.x) * tempFusedData.distances.leftBack / 10.0f + tempFusedData.robotState.position.x - sinf(tempFusedData.robotState.rotation.x + JAFDSettings::Mechanics::distSensLRAngleToMiddle) * JAFDSettings::Mechanics::distSensLRDistToMiddle;

						if (fabs(hitX - tempFusedData.robotState.mapCoordinate.x * JAFDSettings::Field::cellWidth) < JAFDSettings::MazeMapping::widthSecureDetectFactor * JAFDSettings::Field::cellWidth / 2.0f)
						{
							float hitY = cosf(tempFusedData.robotState.rotation.x) * tempFusedData.distances.leftBack / 10.0f + tempFusedData.robotState.position.y + cosf(tempFusedData.robotState.rotation.x + JAFDSettings::Mechanics::distSensLRAngleToMiddle) * JAFDSettings::Mechanics::distSensLRDistToMiddle;

							if (fabs(hitY - tempFusedData.robotState.mapCoordinate.y * JAFDSettings::Field::cellWidth) < JAFDSettings::Field::cellWidth / 2.0f + JAFDSettings::MazeMapping::distLongerThanBorder)
							{
								// Wall is directly in front of us
								leftWallsDetected++;
							}
						}
					}
					else
					{
						float hitY = cosf(tempFusedData.robotState.rotation.x) * tempFusedData.distances.leftBack / 10.0f + tempFusedData.robotState.position.y + cosf(tempFusedData.robotState.rotation.x + JAFDSettings::Mechanics::distSensLRAngleToMiddle) * JAFDSettings::Mechanics::distSensLRDistToMiddle;

						if (fabs(hitY - tempFusedData.robotState.mapCoordinate.y * JAFDSettings::Field::cellWidth) < JAFDSettings::MazeMapping::widthSecureDetectFactor * JAFDSettings::Field::cellWidth / 2.0f)
						{
							float hitX = -sinf(tempFusedData.robotState.rotation.x) * tempFusedData.distances.leftBack / 10.0f + tempFusedData.robotState.position.x - sinf(tempFusedData.robotState.rotation.x + JAFDSettings::Mechanics::distSensLRAngleToMiddle) * JAFDSettings::Mechanics::distSensLRDistToMiddle;

							if (fabs(hitX - tempFusedData.robotState.mapCoordinate.x * JAFDSettings::Field::cellWidth) < JAFDSettings::Field::cellWidth / 2.0f + JAFDSettings::MazeMapping::distLongerThanBorder)
							{
								// Wall is directly in front of us
								leftWallsDetected++;
							}
						}
					}
				}
				else if (tempFusedData.distSensorState.leftBack == DistSensorStatus::underflow)
				{
					leftWallsDetected++;
				}

				if (leftWallsDetected == 2 && tempFusedData.distSensorState.leftFront == DistSensorStatus::ok && tempFusedData.distSensorState.leftBack == DistSensorStatus::ok)
				{
					// Calculate angle if both left distance sensors detected a wall directly left of the robot.
					tempDistSensAngle += asinf((tempFusedData.distances.leftBack - tempFusedData.distances.leftFront) / sqrtf(JAFDSettings::Mechanics::distSensLRSpacing * JAFDSettings::Mechanics::distSensLRSpacing * 100.0f + (tempFusedData.distances.leftBack - tempFusedData.distances.leftFront) * (tempFusedData.distances.leftBack - tempFusedData.distances.leftFront)));
					tempDistSensAngleTrust += 1.0f / 3.0f;
				}

				if (tempFusedData.distSensorState.rightFront == DistSensorStatus::ok)
				{
					// Measurement is ok
					// Check if resulting hit point is a 90° wall in front of us
					if (tempFusedData.robotState.heading == AbsoluteDir::north || tempFusedData.robotState.heading == AbsoluteDir::south)
					{
						float hitX = sinf(tempFusedData.robotState.rotation.x) * tempFusedData.distances.rightFront / 10.0f + tempFusedData.robotState.position.x + sinf(tempFusedData.robotState.rotation.x + JAFDSettings::Mechanics::distSensLRAngleToMiddle) * JAFDSettings::Mechanics::distSensLRDistToMiddle;

						if (fabs(hitX - tempFusedData.robotState.mapCoordinate.x * JAFDSettings::Field::cellWidth) < JAFDSettings::MazeMapping::widthSecureDetectFactor * JAFDSettings::Field::cellWidth / 2.0f)
						{
							float hitY = -cosf(tempFusedData.robotState.rotation.x) * tempFusedData.distances.rightFront / 10.0f + tempFusedData.robotState.position.y - cosf(tempFusedData.robotState.rotation.x + JAFDSettings::Mechanics::distSensLRAngleToMiddle) * JAFDSettings::Mechanics::distSensLRDistToMiddle;

							if (fabs(hitY - tempFusedData.robotState.mapCoordinate.y * JAFDSettings::Field::cellWidth) < JAFDSettings::Field::cellWidth / 2.0f + JAFDSettings::MazeMapping::distLongerThanBorder)
							{
								// Wall is directly in front of us
								rightWallsDetected++;
							}
						}
					}
					else
					{
						float hitY = -cosf(tempFusedData.robotState.rotation.x) * tempFusedData.distances.rightFront / 10.0f + tempFusedData.robotState.position.y - cosf(tempFusedData.robotState.rotation.x + JAFDSettings::Mechanics::distSensLRAngleToMiddle) * JAFDSettings::Mechanics::distSensLRDistToMiddle;

						if (fabs(hitY - tempFusedData.robotState.mapCoordinate.y * JAFDSettings::Field::cellWidth) < JAFDSettings::MazeMapping::widthSecureDetectFactor * JAFDSettings::Field::cellWidth / 2.0f)
						{
							float hitX = sinf(tempFusedData.robotState.rotation.x) * tempFusedData.distances.rightFront / 10.0f + tempFusedData.robotState.position.x + sinf(tempFusedData.robotState.rotation.x + JAFDSettings::Mechanics::distSensLRAngleToMiddle) * JAFDSettings::Mechanics::distSensLRDistToMiddle;

							if (fabs(hitX - tempFusedData.robotState.mapCoordinate.x * JAFDSettings::Field::cellWidth) < JAFDSettings::Field::cellWidth / 2.0f + JAFDSettings::MazeMapping::distLongerThanBorder)
							{
								// Wall is directly in front of us
								rightWallsDetected++;
							}
						}
					}
				}
				else if (tempFusedData.distSensorState.rightFront == DistSensorStatus::underflow)
				{
					rightWallsDetected++;
				}

				if (tempFusedData.distSensorState.rightBack == DistSensorStatus::ok)
				{
					// Measurement is ok
					// Check if resulting hit point is a 90° wall in front of us
					if (tempFusedData.robotState.heading == AbsoluteDir::north || tempFusedData.robotState.heading == AbsoluteDir::south)
					{
						float hitX = sinf(tempFusedData.robotState.rotation.x) * tempFusedData.distances.rightBack / 10.0f + tempFusedData.robotState.position.x + sinf(tempFusedData.robotState.rotation.x - JAFDSettings::Mechanics::distSensLRAngleToMiddle) * JAFDSettings::Mechanics::distSensLRDistToMiddle;

						if (fabs(hitX - tempFusedData.robotState.mapCoordinate.x * JAFDSettings::Field::cellWidth) < JAFDSettings::MazeMapping::widthSecureDetectFactor * JAFDSettings::Field::cellWidth / 2.0f)
						{
							float hitY = -cosf(tempFusedData.robotState.rotation.x) * tempFusedData.distances.rightBack / 10.0f + tempFusedData.robotState.position.y - cosf(tempFusedData.robotState.rotation.x - JAFDSettings::Mechanics::distSensLRAngleToMiddle) * JAFDSettings::Mechanics::distSensLRDistToMiddle;

							if (fabs(hitY - tempFusedData.robotState.mapCoordinate.y * JAFDSettings::Field::cellWidth) < JAFDSettings::Field::cellWidth / 2.0f + JAFDSettings::MazeMapping::distLongerThanBorder)
							{
								// Wall is directly in front of us
								rightWallsDetected++;
							}
						}
					}
					else
					{
						float hitY = -cosf(tempFusedData.robotState.rotation.x) * tempFusedData.distances.rightBack / 10.0f + tempFusedData.robotState.position.y - cosf(tempFusedData.robotState.rotation.x - JAFDSettings::Mechanics::distSensLRAngleToMiddle) * JAFDSettings::Mechanics::distSensLRDistToMiddle;

						if (fabs(hitY - tempFusedData.robotState.mapCoordinate.y * JAFDSettings::Field::cellWidth) < JAFDSettings::MazeMapping::widthSecureDetectFactor * JAFDSettings::Field::cellWidth / 2.0f)
						{
							float hitX = sinf(tempFusedData.robotState.rotation.x) * tempFusedData.distances.rightBack / 10.0f + tempFusedData.robotState.position.x + sinf(tempFusedData.robotState.rotation.x - JAFDSettings::Mechanics::distSensLRAngleToMiddle) * JAFDSettings::Mechanics::distSensLRDistToMiddle;

							if (fabs(hitX - tempFusedData.robotState.mapCoordinate.x * JAFDSettings::Field::cellWidth) < JAFDSettings::Field::cellWidth / 2.0f + JAFDSettings::MazeMapping::distLongerThanBorder)
							{
								// Wall is directly in front of us
								rightWallsDetected++;
							}
						}
					}
				}
				else if (tempFusedData.distSensorState.rightBack == DistSensorStatus::underflow)
				{
					rightWallsDetected++;
				}

				if (rightWallsDetected == 2 && tempFusedData.distSensorState.rightFront == DistSensorStatus::ok && tempFusedData.distSensorState.rightBack == DistSensorStatus::ok)
				{
					// Calculate angle if both right distance sensors detected a wall directly right of the robot.
					tempDistSensAngle += asinf((tempFusedData.distances.rightFront - tempFusedData.distances.rightBack) / sqrtf(JAFDSettings::Mechanics::distSensLRSpacing * JAFDSettings::Mechanics::distSensLRSpacing * 100.0f + (tempFusedData.distances.rightFront - tempFusedData.distances.rightBack) * (tempFusedData.distances.rightFront - tempFusedData.distances.rightBack)));
					tempDistSensAngleTrust += 1.0f / 3.0f;
				}

				if (tempDistSensAngleTrust > 0.0f)
				{
					tempDistSensAngle /= tempDistSensAngleTrust * 3.0f;
				}

				switch (tempFusedData.robotState.heading)
				{
				
				case AbsoluteDir::north:
				{
					if (fmod(tempFusedData.robotState.rotation.x, DEG_TO_RAD * 360.0f) < DEG_TO_RAD * 180.0f)
					{
						distSensAngle = tempFusedData.robotState.rotation.x - fmod(tempFusedData.robotState.rotation.x, DEG_TO_RAD * 360.0f) + tempDistSensAngle;
					}
					else
					{
						distSensAngle = tempFusedData.robotState.rotation.x + DEG_TO_RAD * 360.0f - fmod(tempFusedData.robotState.rotation.x, DEG_TO_RAD * 360.0f) + tempDistSensAngle;
					}

					break;
				}

				case AbsoluteDir::east:
				{
					if (fmod(tempFusedData.robotState.rotation.x - DEG_TO_RAD * 270.0f, DEG_TO_RAD * 360.0f) < DEG_TO_RAD * 180.0f)
					{
						distSensAngle = tempFusedData.robotState.rotation.x - fmod(tempFusedData.robotState.rotation.x - DEG_TO_RAD * 270.0f, DEG_TO_RAD * 360.0f) + tempDistSensAngle;
					}
					else
					{
						distSensAngle = tempFusedData.robotState.rotation.x + DEG_TO_RAD * 360.0f - fmod(tempFusedData.robotState.rotation.x - DEG_TO_RAD * 270.0f, DEG_TO_RAD * 360.0f) + tempDistSensAngle;
					}

					break;
				}

				case AbsoluteDir::south:
				{
					if (fmod(tempFusedData.robotState.rotation.x - DEG_TO_RAD * 180.0f, DEG_TO_RAD * 360.0f) < DEG_TO_RAD * 180.0f)
					{
						distSensAngle = tempFusedData.robotState.rotation.x - fmod(tempFusedData.robotState.rotation.x - DEG_TO_RAD * 180.0f, DEG_TO_RAD * 360.0f) + tempDistSensAngle;
					}
					else
					{
						distSensAngle = tempFusedData.robotState.rotation.x + DEG_TO_RAD * 360.0f - fmod(tempFusedData.robotState.rotation.x - DEG_TO_RAD * 180.0f, DEG_TO_RAD * 360.0f) + tempDistSensAngle;
					}

					break;
				}

				case AbsoluteDir::west:
				{
					if (fmod(tempFusedData.robotState.rotation.x - DEG_TO_RAD * 90.0f, DEG_TO_RAD * 360.0f) < DEG_TO_RAD * 180.0f)
					{
						distSensAngle = tempFusedData.robotState.rotation.x - fmod(tempFusedData.robotState.rotation.x - DEG_TO_RAD * 90.0f, DEG_TO_RAD * 360.0f) + tempDistSensAngle;
					}
					else
					{
						distSensAngle = tempFusedData.robotState.rotation.x + DEG_TO_RAD * 360.0f - fmod(tempFusedData.robotState.rotation.x - DEG_TO_RAD * 90.0f, DEG_TO_RAD * 360.0f) + tempDistSensAngle;
					}

					break;
				}

				default:
					break;
				}

				distSensAngleTrust = tempDistSensAngleTrust;

				//if (tempFusedData.distSensorState.frontLong == DistSensorStatus::ok)
				//{
				//	bool hitPointIsOk = false;

				//	// Measurement is ok
				//	// Check if resulting hit point is a 90° wall in front of us
				//	if (tempFusedData.robotState.heading == AbsoluteDir::north || tempFusedData.robotState.heading == AbsoluteDir::south)
				//	{
				//		float hitY = sinf(tempFusedData.robotState.rotation.x) * (tempFusedData.distances.frontLong / 10.0f + JAFDSettings::Mechanics::distSensFrontBackDist / 2.0f) + tempFusedData.robotState.position.y;

				//		if (fabs(hitY - tempFusedData.robotState.mapCoordinate.y * JAFDSettings::Field::cellWidth) < JAFDSettings::MazeMapping::widthSecureDetectFactor * JAFDSettings::Field::cellWidth / 2.0f)
				//		{
				//			hitPointIsOk = true;

				//			float hitX = cosf(tempFusedData.robotState.rotation.x) * (tempFusedData.distances.frontRight / 10.0f + JAFDSettings::Mechanics::distSensFrontBackDist / 2.0f) + tempFusedData.robotState.position.x;
				//		}
				//	}
				//	else
				//	{
				//		float hitX = cosf(tempFusedData.robotState.rotation.x) * (tempFusedData.distances.frontRight / 10.0f + JAFDSettings::Mechanics::distSensFrontBackDist / 2.0f) + tempFusedData.robotState.position.x;

				//		if (fabs(hitX - tempFusedData.robotState.mapCoordinate.x * JAFDSettings::Field::cellWidth) < JAFDSettings::MazeMapping::widthSecureDetectFactor * JAFDSettings::Field::cellWidth / 2.0f)
				//		{
				//			hitPointIsOk = true;
				//			
				//			float hitY = sinf(tempFusedData.robotState.rotation.x) * (tempFusedData.distances.frontLong / 10.0f + JAFDSettings::Mechanics::distSensFrontBackDist / 2.0f) + tempFusedData.robotState.position.y;
				//		}
				//	}

				//	if (hitPointIsOk)
				//	{
				//		if (lastMiddleFrontDist != 0 && lastTime != 0)
				//		{
				//			tempDistSensSpeed += (tempFusedData.distances.frontLong - lastMiddleFrontDist) / 10.0f * 1000.0f / (now - lastTime);
				//			validDistSpeedSamples++;
				//		}

				//		lastMiddleFrontDist = tempFusedData.distances.frontLong;
				//	}
				//	else
				//	{
				//		lastMiddleFrontDist = 0;
				//	}
				//}

				//if (tempFusedData.distSensorState.backLong == DistSensorStatus::ok)
				//{
				//	bool hitPointIsOk = false;

				//	// Measurement is ok
				//	// Check if resulting hit point is a 90° wall in front of us
				//	if (tempFusedData.robotState.heading == AbsoluteDir::north || tempFusedData.robotState.heading == AbsoluteDir::south)
				//	{
				//		float hitY = -sinf(tempFusedData.robotState.rotation.x) * (tempFusedData.distances.frontLong / 10.0f + JAFDSettings::Mechanics::distSensFrontBackDist / 2.0f) + tempFusedData.robotState.position.y;

				//		if (fabs(hitY - tempFusedData.robotState.mapCoordinate.y * JAFDSettings::Field::cellWidth) < JAFDSettings::MazeMapping::widthSecureDetectFactor * JAFDSettings::Field::cellWidth / 2.0f)
				//		{
				//			hitPointIsOk = true;
				//		}
				//	}
				//	else
				//	{
				//		float hitX = -cosf(tempFusedData.robotState.rotation.x) * (tempFusedData.distances.frontRight / 10.0f + JAFDSettings::Mechanics::distSensFrontBackDist / 2.0f) + tempFusedData.robotState.position.x;

				//		if (fabs(hitX - tempFusedData.robotState.mapCoordinate.x * JAFDSettings::Field::cellWidth) < JAFDSettings::MazeMapping::widthSecureDetectFactor * JAFDSettings::Field::cellWidth / 2.0f)
				//		{
				//			hitPointIsOk = true;
				//		}
				//	}

				//	if (hitPointIsOk)
				//	{
				//		if (lastMiddleBackDist != 0 && lastTime != 0)
				//		{
				//			tempDistSensSpeed -= (tempFusedData.distances.backLong - lastMiddleBackDist) / 10.0f * 1000.0f / (now - lastTime); // Negative, because it is the back-sensor
				//			validDistSpeedSamples++;
				//		}

				//		lastMiddleBackDist = tempFusedData.distances.backLong;
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
				distSensSpeedTrust = validDistSpeedSamples / 4.0f;
				distSensSpeed = (-tempDistSensSpeed / (float)(validDistSpeedSamples)) * JAFDSettings::SensorFusion::distSensSpeedIIRFactor + distSensSpeed * (1.0f - JAFDSettings::SensorFusion::distSensSpeedIIRFactor);	// Negative, because increasing distance means driving away; 
			}
			else
			{
				distSensSpeedTrust = 0.0f;
			}

			lastTime = now;
			fusedData = tempFusedData;
		}

		void updateDistSensor()
		{
			uint16_t tempDist = 0.0f;
			uint32_t tempAverageDist = 0;
			uint8_t numCorrectSamples = 0;
			uint8_t numOverflowSamples = 0;
			uint8_t numUnderflowSamples = 0;

			FusedData tempFusedData = fusedData;

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
				if (tempFusedData.distSensorState.frontLeft == DistSensorStatus::ok)
				{
					tempFusedData.distances.frontLeft = static_cast<uint16_t>((tempAverageDist / numCorrectSamples) * JAFDSettings::SensorFusion::shortDistSensIIRFactor + tempFusedData.distances.frontLeft * (1.0f - JAFDSettings::SensorFusion::shortDistSensIIRFactor));
				}
				else
				{
					tempFusedData.distances.frontLeft = static_cast<uint16_t>(tempAverageDist / numCorrectSamples);
				}

				tempFusedData.distSensorState.frontLeft = DistSensorStatus::ok;
			}
			else
			{
				if (numOverflowSamples > (JAFDSettings::DistanceSensors::averagingNumSamples - numOverflowSamples))
				{
					tempFusedData.distSensorState.frontLeft = DistSensorStatus::overflow;
				}
				else if (numUnderflowSamples > (JAFDSettings::DistanceSensors::averagingNumSamples - numUnderflowSamples))
				{
					tempFusedData.distSensorState.frontLeft = DistSensorStatus::underflow;
				}
				else
				{
					tempFusedData.distSensorState.frontLeft = DistSensorStatus::error;
				}

				tempFusedData.distances.frontLeft = 0;
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
				if (tempFusedData.distSensorState.frontRight == DistSensorStatus::ok)
				{
					tempFusedData.distances.frontRight = static_cast<uint16_t>((tempAverageDist / numCorrectSamples) * JAFDSettings::SensorFusion::shortDistSensIIRFactor + tempFusedData.distances.frontRight * (1.0f - JAFDSettings::SensorFusion::shortDistSensIIRFactor));
				}
				else
				{
					tempFusedData.distances.frontRight = static_cast<uint16_t>(tempAverageDist / numCorrectSamples);
				}

				tempFusedData.distSensorState.frontRight = DistSensorStatus::ok;
			}
			else
			{
				if (numOverflowSamples > (JAFDSettings::DistanceSensors::averagingNumSamples - numOverflowSamples))
				{
					tempFusedData.distSensorState.frontRight = DistSensorStatus::overflow;
				}
				else if (numUnderflowSamples > (JAFDSettings::DistanceSensors::averagingNumSamples - numUnderflowSamples))
				{
					tempFusedData.distSensorState.frontRight = DistSensorStatus::underflow;
				}
				else
				{
					tempFusedData.distSensorState.frontRight = DistSensorStatus::error;
				}

				tempFusedData.distances.frontRight = 0;
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
				if (tempFusedData.distSensorState.leftBack == DistSensorStatus::ok)
				{
					tempFusedData.distances.leftBack = static_cast<uint16_t>((tempAverageDist / numCorrectSamples) * JAFDSettings::SensorFusion::shortDistSensIIRFactor + tempFusedData.distances.leftBack * (1.0f - JAFDSettings::SensorFusion::shortDistSensIIRFactor));
				}
				else
				{
					tempFusedData.distances.leftBack = static_cast<uint16_t>(tempAverageDist / numCorrectSamples);
				}

				tempFusedData.distSensorState.leftBack = DistSensorStatus::ok;
			}
			else
			{
				if (numOverflowSamples > (JAFDSettings::DistanceSensors::averagingNumSamples - numOverflowSamples))
				{
					tempFusedData.distSensorState.leftBack = DistSensorStatus::overflow;
				}
				else if (numUnderflowSamples > (JAFDSettings::DistanceSensors::averagingNumSamples - numUnderflowSamples))
				{
					tempFusedData.distSensorState.leftBack = DistSensorStatus::underflow;
				}
				else
				{
					tempFusedData.distSensorState.leftBack = DistSensorStatus::error;
				}

				tempFusedData.distances.leftBack = 0;
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
				if (tempFusedData.distSensorState.leftFront == DistSensorStatus::ok)
				{
					tempFusedData.distances.leftFront = static_cast<uint16_t>((tempAverageDist / numCorrectSamples) * JAFDSettings::SensorFusion::shortDistSensIIRFactor + tempFusedData.distances.leftFront * (1.0f - JAFDSettings::SensorFusion::shortDistSensIIRFactor));
				}
				else
				{
					tempFusedData.distances.leftFront = static_cast<uint16_t>(tempAverageDist / numCorrectSamples);
				}

				tempFusedData.distSensorState.leftFront = DistSensorStatus::ok;
			}
			else
			{
				if (numOverflowSamples > (JAFDSettings::DistanceSensors::averagingNumSamples - numOverflowSamples))
				{
					tempFusedData.distSensorState.leftFront = DistSensorStatus::overflow;
				}
				else if (numUnderflowSamples > (JAFDSettings::DistanceSensors::averagingNumSamples - numUnderflowSamples))
				{
					tempFusedData.distSensorState.leftFront = DistSensorStatus::underflow;
				}
				else
				{
					tempFusedData.distSensorState.leftFront = DistSensorStatus::error;
				}

				tempFusedData.distances.leftFront = 0;
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
				if (tempFusedData.distSensorState.rightBack == DistSensorStatus::ok)
				{
					tempFusedData.distances.rightBack = static_cast<uint16_t>((tempAverageDist / numCorrectSamples) * JAFDSettings::SensorFusion::shortDistSensIIRFactor + tempFusedData.distances.rightBack * (1.0f - JAFDSettings::SensorFusion::shortDistSensIIRFactor));
				}
				else
				{
					tempFusedData.distances.rightBack = static_cast<uint16_t>(tempAverageDist / numCorrectSamples);
				}

				tempFusedData.distSensorState.rightBack = DistSensorStatus::ok;
			}
			else
			{
				if (numOverflowSamples > (JAFDSettings::DistanceSensors::averagingNumSamples - numOverflowSamples))
				{
					tempFusedData.distSensorState.rightBack = DistSensorStatus::overflow;
				}
				else if (numUnderflowSamples > (JAFDSettings::DistanceSensors::averagingNumSamples - numUnderflowSamples))
				{
					tempFusedData.distSensorState.rightBack = DistSensorStatus::underflow;
				}
				else
				{
					tempFusedData.distSensorState.rightBack = DistSensorStatus::error;
				}

				tempFusedData.distances.rightBack = 0;
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
				if (tempFusedData.distSensorState.rightFront == DistSensorStatus::ok)
				{
					tempFusedData.distances.rightFront = static_cast<uint16_t>((tempAverageDist / numCorrectSamples) * JAFDSettings::SensorFusion::shortDistSensIIRFactor + tempFusedData.distances.rightFront * (1.0f - JAFDSettings::SensorFusion::shortDistSensIIRFactor));
				}
				else
				{
					tempFusedData.distances.rightFront = static_cast<uint16_t>(tempAverageDist / numCorrectSamples);
				}

				tempFusedData.distSensorState.rightFront = DistSensorStatus::ok;
			}
			else
			{
				if (numOverflowSamples > (JAFDSettings::DistanceSensors::averagingNumSamples - numOverflowSamples))
				{
					tempFusedData.distSensorState.rightFront = DistSensorStatus::overflow;
				}
				else if (numUnderflowSamples > (JAFDSettings::DistanceSensors::averagingNumSamples - numUnderflowSamples))
				{
					tempFusedData.distSensorState.rightFront = DistSensorStatus::underflow;
				}
				else
				{
					tempFusedData.distSensorState.rightFront = DistSensorStatus::error;
				}

				tempFusedData.distances.rightFront = 0;
			}

			numCorrectSamples = 0;
			tempAverageDist = 0;
			numOverflowSamples = 0;
			numUnderflowSamples = 0;

			//for (uint8_t i = 0; i < JAFDSettings::DistanceSensors::averagingNumSamples; i++)
			//{
			//	tempDist = DistanceSensors::frontLong.getDistance();

			//	if (DistanceSensors::frontLong.getStatus() == decltype(DistanceSensors::frontLong)::Status::noError)
			//	{
			//		numCorrectSamples++;
			//		tempAverageDist += tempDist;
			//	}
			//	else if (DistanceSensors::frontLong.getStatus() == decltype(DistanceSensors::frontLong)::Status::overflow)
			//	{
			//		numOverflowSamples++;
			//	}
			//	else if (DistanceSensors::frontLong.getStatus() == decltype(DistanceSensors::frontLong)::Status::underflow)
			//	{
			//		numUnderflowSamples++;
			//	}
			//}

			//if (numCorrectSamples > (JAFDSettings::DistanceSensors::averagingNumSamples - numCorrectSamples))
			//{
			//	if (tempFusedData.distSensorState.frontLong == DistSensorStatus::ok)
			//	{
			//		tempFusedData.distances.frontLong = static_cast<uint16_t>((tempAverageDist / numCorrectSamples) * JAFDSettings::SensorFusion::longDistSensIIRFactor + tempFusedData.distances.frontLong * (1.0f - JAFDSettings::SensorFusion::longDistSensIIRFactor));
			//	}
			//	else
			//	{
			//		tempFusedData.distances.frontLong = static_cast<uint16_t>(tempAverageDist / numCorrectSamples);
			//	}

			//	tempFusedData.distSensorState.frontLong = DistSensorStatus::ok;
			//}
			//else
			//{
			//	if (numOverflowSamples > (JAFDSettings::DistanceSensors::averagingNumSamples - numOverflowSamples))
			//	{
			//		tempFusedData.distSensorState.frontLong = DistSensorStatus::overflow;
			//	}
			//	else if (numUnderflowSamples > (JAFDSettings::DistanceSensors::averagingNumSamples - numUnderflowSamples))
			//	{
			//		tempFusedData.distSensorState.frontLong = DistSensorStatus::underflow;
			//	}
			//	else
			//	{
			//		tempFusedData.distSensorState.frontLong = DistSensorStatus::error;
			//	}

			//	tempFusedData.distances.frontLong = 0;
			//}

			//numCorrectSamples = 0;
			//tempAverageDist = 0;
			//numOverflowSamples = 0;
			//numUnderflowSamples = 0;

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
			//	if (tempFusedData.distSensorState.backLong == DistSensorStatus::ok)
			//	{
			//		tempFusedData.distances.backLong = static_cast<uint16_t>((tempAverageDist / numCorrectSamples) * JAFDSettings::SensorFusion::longDistSensIIRFactor + tempFusedData.distances.backLong * (1.0f - JAFDSettings::SensorFusion::longDistSensIIRFactor));
			//	}
			//	else
			//	{
			//		tempFusedData.distances.backLong = static_cast<uint16_t>(tempAverageDist / numCorrectSamples);
			//	}

			//	tempFusedData.distSensorState.backLong = DistSensorStatus::ok;
			//}
			//else
			//{
			//	if (numOverflowSamples > (JAFDSettings::DistanceSensors::averagingNumSamples - numOverflowSamples))
			//	{
			//		tempFusedData.distSensorState.backLong = DistSensorStatus::overflow;
			//	}
			//	else if (numUnderflowSamples > (JAFDSettings::DistanceSensors::averagingNumSamples - numUnderflowSamples))
			//	{
			//		tempFusedData.distSensorState.backLong = DistSensorStatus::underflow;
			//	}
			//	else
			//	{
			//		tempFusedData.distSensorState.backLong = DistSensorStatus::error;
			//	}

			//	tempFusedData.distances.backLong = 0;
			//}

			fusedData = tempFusedData;
		}

		void setCertainRobotPosition(Vec3f pos, Vec3f rotation)
		{
			fusedData.robotState.rotation.y = rotation.y;
			fusedData.robotState.rotation.z = rotation.z;

			float headingOffset = fusedData.robotState.rotation.x - rotation.x;

			while (headingOffset < 0.0f) headingOffset += M_TWOPI;
			while (headingOffset > M_PI) headingOffset -= M_TWOPI;

			totalHeadingOff += headingOffset;

			fusedData.robotState.rotation.x -= headingOffset;

			fusedData.robotState.position = pos;

			fusedData.robotState.mapCoordinate.x = roundf(fusedData.robotState.position.x / JAFDSettings::Field::cellWidth);
			fusedData.robotState.mapCoordinate.y = roundf(fusedData.robotState.position.y / JAFDSettings::Field::cellWidth);
			fusedData.robotState.mapCoordinate.floor = 0;

			if (fusedData.robotState.rotation.x > 315.0f * DEG_TO_RAD || fusedData.robotState.rotation.x <= 45.0f * DEG_TO_RAD) fusedData.robotState.heading = AbsoluteDir::north;
			else if (fusedData.robotState.rotation.x > 45.0f * DEG_TO_RAD && fusedData.robotState.rotation.x <= 135.0f * DEG_TO_RAD) fusedData.robotState.heading = AbsoluteDir::west;
			else if (fusedData.robotState.rotation.x > 135.0f * DEG_TO_RAD && fusedData.robotState.rotation.x <= 225.0f * DEG_TO_RAD) fusedData.robotState.heading = AbsoluteDir::south;
			else fusedData.robotState.heading = AbsoluteDir::east;
		}

		const volatile FusedData& getFusedData()
		{
			return fusedData;
		}
	}
}