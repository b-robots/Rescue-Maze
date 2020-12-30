/*
This file of the library is responsible for the sensor fusion
*/

#pragma once

#include "../header/MazeMapping.h"
#include "../header/Math.h"
#include "../header/SensorFusion.h"
#include "../header/MotorControl.h"
#include "../header/DistanceSensors.h"
#include "../header/Bno055.h"
#include "../header/TCS34725.h"
#include "../header/RobotLogic.h"
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
			volatile float distSensAngle = 0.0f;		// Angle measured by distance sensors (rad)
			volatile float distSensAngleTrust = 0.0f;	// How much can I trust the measured angle? (0.0 - 1.0)
		}

		void sensorFiltering(const uint8_t freq)
		{
			RobotState tempRobotState = fusedData.robotState;

			tempRobotState.wheelSpeeds = MotorControl::getFloatSpeeds();
			
			// Magic factor: *1.17

			// Angular velocity
			static Vec3f prevBnoAngVel;
			auto prevAngularVel = tempRobotState.angularVel;

			float encoderYawVel = (tempRobotState.wheelSpeeds.right - tempRobotState.wheelSpeeds.left) / (JAFDSettings::Mechanics::wheelDistToMiddle * 2.0f * 1.17f);
			auto bnoAngVel = Bno055::get_angular_velocity();
			bnoAngVel = prevBnoAngVel * (1.0f - JAFDSettings::SensorFusion::bno055AngularVelIIr) + bnoAngVel * JAFDSettings::SensorFusion::bno055AngularVelIIr;
			prevBnoAngVel = bnoAngVel;

			tempRobotState.angularVel.x = bnoAngVel.x * JAFDSettings::SensorFusion::bno055AngularVelPortion + encoderYawVel * (1.0f - JAFDSettings::SensorFusion::bno055AngularVelPortion);
			tempRobotState.angularVel.y = bnoAngVel.y;
			tempRobotState.angularVel.z = bnoAngVel.z;
			
			tempRobotState.angularVel = prevAngularVel * (1.0f - JAFDSettings::SensorFusion::angularVelIIRFactor) + tempRobotState.angularVel * JAFDSettings::SensorFusion::angularVelIIRFactor;

			// Relative angle
			Vec3f prevRot = tempRobotState.rotation;

			float encoderYawAngle = (MotorControl::getDistance(Motor::right) - MotorControl::getDistance(Motor::left)) / (JAFDSettings::Mechanics::wheelDistToMiddle * 2.0f * 1.17f);
			encoderYawAngle = fitAnglesToInterval(encoderYawAngle - totalHeadingOff);
			auto bnoAbsOr = fitAnglesToInterval(Bno055::get_absolute_orientation() - Vec3f(totalHeadingOff, 0, 0));

			float combinedYaw = interpolateAngles(encoderYawAngle, fitAnglesToInterval(distSensAngle), distSensAngleTrust * JAFDSettings::SensorFusion::distAngularPortion);

			tempRobotState.rotation.x = interpolateAngles(combinedYaw, fitAnglesToInterval(bnoAbsOr.x), JAFDSettings::SensorFusion::bno055RotPortion);
			tempRobotState.rotation.y = interpolateAngles(tempRobotState.rotation.y, fitAnglesToInterval(bnoAbsOr.y), JAFDSettings::SensorFusion::pitchIIRFactor);
			tempRobotState.rotation.z = bnoAbsOr.z;

			auto integratedAngle = fitAnglesToInterval(prevRot + tempRobotState.angularVel / (float)freq);
			tempRobotState.rotation = interpolateAngles(tempRobotState.rotation, integratedAngle, JAFDSettings::SensorFusion::rotationIntegralFactor);

			tempRobotState.rotation = makeRotationCoherent(prevRot, tempRobotState.rotation);

			// Linear velocitys
			tempRobotState.forwardVel = ((tempRobotState.wheelSpeeds.left + tempRobotState.wheelSpeeds.right) / 2.0f) * (1.0f - distSensSpeedTrust * JAFDSettings::SensorFusion::distSpeedPortion) + distSensSpeed * (distSensSpeedTrust * JAFDSettings::SensorFusion::distSpeedPortion);

			// Position
			tempRobotState.position += (Vec3f::angleToDir(tempRobotState.rotation.x, tempRobotState.rotation.y) * tempRobotState.forwardVel / freq);

			// Map coordinates
			tempRobotState.mapCoordinate.x = roundf(tempRobotState.position.x / JAFDSettings::Field::cellWidth);
			tempRobotState.mapCoordinate.y = roundf(tempRobotState.position.y / JAFDSettings::Field::cellWidth);

			// Heading
			float positiveAngle = tempRobotState.rotation.x;

			while (positiveAngle < 0.0f) positiveAngle += M_TWOPI;
			while (positiveAngle > M_TWOPI) positiveAngle -= M_TWOPI;

			if (RAD_TO_DEG * positiveAngle > 315.0f || RAD_TO_DEG * positiveAngle < 45.0f) tempRobotState.heading = AbsoluteDir::north;
			else if (RAD_TO_DEG * positiveAngle > 45.0f && RAD_TO_DEG * positiveAngle < 135.0f) tempRobotState.heading = AbsoluteDir::west;
			else if (RAD_TO_DEG * positiveAngle > 135.0f && RAD_TO_DEG * positiveAngle < 225.0f) tempRobotState.heading = AbsoluteDir::south;
			else tempRobotState.heading = AbsoluteDir::east;

			fusedData.robotState = tempRobotState;
		}

		void untimedFusion()
		{
			static uint32_t lastTime = 0;
			uint32_t now = millis();

			auto tempFusedData = fusedData;
			
			// Speed measurement with distances
			uint8_t validDistSpeedSamples = 0;			// Number of valid speed measurements by distance sensor
			static uint16_t lastLeftDist = 0;			// Last distance left
			static uint16_t lastRightDist = 0;			// Last distance right
			static uint16_t lastMiddleFrontDist = 0;	// Last distance middle front
			static uint16_t lastMiddleBackDist = 0;		// Last distance middle back
			float tempDistSensSpeed = 0.0f;				// Measured speed 

			// Angle measurement with distances
			float tempDistSensAngle = 0.0f;				// Angle measured by distance sensors
			float tempDistSensAngleTrust = 0.0f;		// How much can I trust the measured angle? (0.0 - 1.0)

			// Wall detection
			uint8_t frontWallsDetected = 0;		// How many times did a wall in front of us get detected
			uint8_t leftWallsDetected = 0;		// How many times did a wall left of us get detected
			uint8_t rightWallsDetected = 0;		// How many times did a wall right of us get detected

			// MazeMapping
			static MapCoordinate lastDifferentPosittion = homePosition;
			static MapCoordinate lastPosition = homePosition;
			GridCell tempCell;
			float updateCertainty = 1.0f;
			tempCell.cellState = CellState::visited;
			tempCell.cellConnections = Directions::nowhere;
			uint8_t walls = 0b0000;											// Where are the walls; inverted to cellConnections

			if (lastPosition != tempFusedData.robotState.mapCoordinate)
			{
				lastDifferentPosittion = lastPosition;

				MazeMapping::getGridCell(&tempFusedData.gridCell, tempFusedData.robotState.mapCoordinate);

				if (tempFusedData.gridCell.cellState & CellState::visited)
				{
					tempFusedData.gridCellCertainty = 0.6f;
					tempCell.cellState = tempFusedData.gridCell.cellState;
				}
				else
				{
					tempFusedData.gridCellCertainty = 0.0f;
				}
			}

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

				if (tempFusedData.distSensorState.frontLong == DistSensorStatus::ok)
				{
					bool hitPointIsOk = false;

					// Measurement is ok
					// Check if resulting hit point is a 90° wall in front of us
					if (tempFusedData.robotState.heading == AbsoluteDir::north || tempFusedData.robotState.heading == AbsoluteDir::south)
					{
						float hitY = sinf(tempFusedData.robotState.rotation.x) * (tempFusedData.distances.frontLong / 10.0f + JAFDSettings::Mechanics::distSensFrontBackDist / 2.0f) + tempFusedData.robotState.position.y;

						if (fabs(hitY - tempFusedData.robotState.mapCoordinate.y * JAFDSettings::Field::cellWidth) < JAFDSettings::MazeMapping::widthSecureDetectFactor * JAFDSettings::Field::cellWidth / 2.0f)
						{
							hitPointIsOk = true;

							float hitX = cosf(tempFusedData.robotState.rotation.x) * (tempFusedData.distances.frontRight / 10.0f + JAFDSettings::Mechanics::distSensFrontBackDist / 2.0f) + tempFusedData.robotState.position.x;
						}
					}
					else
					{
						float hitX = cosf(tempFusedData.robotState.rotation.x) * (tempFusedData.distances.frontRight / 10.0f + JAFDSettings::Mechanics::distSensFrontBackDist / 2.0f) + tempFusedData.robotState.position.x;

						if (fabs(hitX - tempFusedData.robotState.mapCoordinate.x * JAFDSettings::Field::cellWidth) < JAFDSettings::MazeMapping::widthSecureDetectFactor * JAFDSettings::Field::cellWidth / 2.0f)
						{
							hitPointIsOk = true;
							
							float hitY = sinf(tempFusedData.robotState.rotation.x) * (tempFusedData.distances.frontLong / 10.0f + JAFDSettings::Mechanics::distSensFrontBackDist / 2.0f) + tempFusedData.robotState.position.y;
						}
					}

					if (hitPointIsOk)
					{
						if (lastMiddleFrontDist != 0 && lastTime != 0)
						{
							tempDistSensSpeed += (tempFusedData.distances.frontLong - lastMiddleFrontDist) / 10.0f * 1000.0f / (now - lastTime);
							validDistSpeedSamples++;
						}

						lastMiddleFrontDist = tempFusedData.distances.frontLong;
					}
					else
					{
						lastMiddleFrontDist = 0;
					}
				}
			}
			else
			{
				lastLeftDist = 0;
				lastRightDist = 0;
				lastMiddleFrontDist = 0;
				lastMiddleBackDist = 0;
			}

			if (frontWallsDetected > 0)
			{
				switch (makeAbsolute(RelativeDir::forward, tempFusedData.robotState.heading))
				{

				case AbsoluteDir::north:
				{
					if ((~tempFusedData.gridCell.cellConnections & Directions::north && tempFusedData.gridCell.cellState & CellState::visited) || frontWallsDetected > 1)
					{
						walls |= Directions::north;
					}
					else if (!(tempFusedData.gridCell.cellState & CellState::visited))
					{
						walls |= Directions::north;
						updateCertainty -= 0.08f;
					}
					else if (frontWallsDetected < 2)
					{
						updateCertainty -= 0.15f;
					}
					else
					{
						updateCertainty -= 0.25f;
					}

					break;
				}

				case AbsoluteDir::east:
				{
					if ((~tempFusedData.gridCell.cellConnections & Directions::east && tempFusedData.gridCell.cellState & CellState::visited) || frontWallsDetected > 1)
					{
						walls |= Directions::east;
					}
					else if (!(tempFusedData.gridCell.cellState & CellState::visited))
					{
						walls |= Directions::east;
						updateCertainty -= 0.08f;
					}
					else if (frontWallsDetected < 2)
					{
						updateCertainty -= 0.15f;
					}
					else
					{
						updateCertainty -= 0.25f;
					}

					break;
				}

				case AbsoluteDir::south:
				{
					if ((~tempFusedData.gridCell.cellConnections & Directions::south && tempFusedData.gridCell.cellState & CellState::visited) || frontWallsDetected > 1)
					{
						walls |= Directions::south;
					}
					else if (!(tempFusedData.gridCell.cellState & CellState::visited))
					{
						walls |= Directions::south;
						updateCertainty -= 0.08f;
					}
					else if (frontWallsDetected < 2)
					{
						updateCertainty -= 0.15f;
					}
					else
					{
						updateCertainty -= 0.25f;
					}

					break;
				}

				case AbsoluteDir::west:
				{
					if ((~tempFusedData.gridCell.cellConnections & Directions::west && tempFusedData.gridCell.cellState & CellState::visited) || frontWallsDetected > 1)
					{
						walls |= Directions::west;
					}
					else if (!(tempFusedData.gridCell.cellState & CellState::visited))
					{
						walls |= Directions::west;
						updateCertainty -= 0.08f;
					}
					else if (frontWallsDetected < 2)
					{
						updateCertainty -= 0.15f;
					}
					else
					{
						updateCertainty -= 0.25f;
					}

					break;
				}

				default:
					break;
				}
			}

			if (leftWallsDetected > 0)
			{
				switch (makeAbsolute(RelativeDir::left, tempFusedData.robotState.heading))
				{

				case AbsoluteDir::north:
				{
					if ((~tempFusedData.gridCell.cellConnections & Directions::north && tempFusedData.gridCell.cellState & CellState::visited) || leftWallsDetected > 1)
					{
						walls |= Directions::north;
					}
					else if (!(tempFusedData.gridCell.cellState & CellState::visited))
					{
						walls |= Directions::north;
						updateCertainty -= 0.08f;
					}
					else if (leftWallsDetected < 2)
					{
						updateCertainty -= 0.15f;
					}
					else
					{
						updateCertainty -= 0.25f;
					}

					break;
				}

				case AbsoluteDir::east:
				{
					if ((~tempFusedData.gridCell.cellConnections & Directions::east && tempFusedData.gridCell.cellState & CellState::visited) || leftWallsDetected > 1)
					{
						walls |= Directions::east;
					}
					else if (!(tempFusedData.gridCell.cellState & CellState::visited))
					{
						walls |= Directions::east;
						updateCertainty -= 0.08f;
					}
					else if (leftWallsDetected < 2)
					{
						updateCertainty -= 0.15f;
					}
					else
					{
						updateCertainty -= 0.25f;
					}

					break;
				}

				case AbsoluteDir::south:
				{
					if ((~tempFusedData.gridCell.cellConnections & Directions::south && tempFusedData.gridCell.cellState & CellState::visited) || leftWallsDetected > 1)
					{
						walls |= Directions::south;
					}
					else if (!(tempFusedData.gridCell.cellState & CellState::visited))
					{
						walls |= Directions::south;
						updateCertainty -= 0.08f;
					}
					else if (leftWallsDetected < 2)
					{
						updateCertainty -= 0.15f;
					}
					else
					{
						updateCertainty -= 0.25f;
					}

					break;
				}

				case AbsoluteDir::west:
				{
					if ((~tempFusedData.gridCell.cellConnections & Directions::west && tempFusedData.gridCell.cellState & CellState::visited) || leftWallsDetected > 1)
					{
						walls |= Directions::west;
					}
					else if (!(tempFusedData.gridCell.cellState & CellState::visited))
					{
						walls |= Directions::west;
						updateCertainty -= 0.08f;
					}
					else if (leftWallsDetected < 2)
					{
						updateCertainty -= 0.15f;
					}
					else
					{
						updateCertainty -= 0.25f;
					}

					break;
				}

				default:
					break;
				}
			}

			if (rightWallsDetected > 0)
			{
				switch (makeAbsolute(RelativeDir::right, tempFusedData.robotState.heading))
				{

				case AbsoluteDir::north:
				{
					if ((~tempFusedData.gridCell.cellConnections & Directions::north && tempFusedData.gridCell.cellState & CellState::visited) || rightWallsDetected > 1)
					{
						walls |= Directions::north;
					}
					else if (!(tempFusedData.gridCell.cellState & CellState::visited))
					{
						walls |= Directions::north;
						updateCertainty -= 0.08f;
					}
					else if (rightWallsDetected < 2)
					{
						updateCertainty -= 0.15f;
					}
					else
					{
						updateCertainty -= 0.25f;
					}

					break;
				}

				case AbsoluteDir::east:
				{
					if ((~tempFusedData.gridCell.cellConnections & Directions::east && tempFusedData.gridCell.cellState & CellState::visited) || rightWallsDetected > 1)
					{
						walls |= Directions::east;
					}
					else if (!(tempFusedData.gridCell.cellState & CellState::visited))
					{
						walls |= Directions::east;
						updateCertainty -= 0.08f;
					}
					else if (rightWallsDetected < 2)
					{
						updateCertainty -= 0.15f;
					}
					else
					{
						updateCertainty -= 0.25f;
					}

					break;
				}

				case AbsoluteDir::south:
				{
					if ((~tempFusedData.gridCell.cellConnections & Directions::south && tempFusedData.gridCell.cellState & CellState::visited) || rightWallsDetected > 1)
					{
						walls |= Directions::south;
					}
					else if (!(tempFusedData.gridCell.cellState & CellState::visited))
					{
						walls |= Directions::south;
						updateCertainty -= 0.08f;
					}
					else if (rightWallsDetected < 2)
					{
						updateCertainty -= 0.15f;
					}
					else
					{
						updateCertainty -= 0.25f;
					}

					break;
				}

				case AbsoluteDir::west:
				{
					if ((~tempFusedData.gridCell.cellConnections & Directions::west && tempFusedData.gridCell.cellState & CellState::visited) || rightWallsDetected > 1)
					{
						walls |= Directions::west;
					}
					else if (!(tempFusedData.gridCell.cellState & CellState::visited))
					{
						walls |= Directions::west;
						updateCertainty -= 0.08f;
					}
					else if (rightWallsDetected < 2)
					{
						updateCertainty -= 0.15f;
					}
					else
					{
						updateCertainty -= 0.25f;
					}

					break;
				}

				default:
					break;
				}
			}

			if (tempFusedData.robotState.mapCoordinate.x > lastDifferentPosittion.x)
			{
				walls |= Directions::south;

				if (!(~tempFusedData.gridCell.cellConnections & Directions::south) && tempFusedData.gridCell.cellState & CellState::visited)
				{
					updateCertainty -= 0.25f;
				}
			}
			else if (tempFusedData.robotState.mapCoordinate.x < lastDifferentPosittion.x)
			{
				walls |= Directions::north;

				if (!(~tempFusedData.gridCell.cellConnections & Directions::north) && tempFusedData.gridCell.cellState & CellState::visited)
				{
					updateCertainty -= 0.25f;
				}
			}

			if (tempFusedData.robotState.mapCoordinate.y > lastDifferentPosittion.y)
			{
				walls |= Directions::east;

				if (!(~tempFusedData.gridCell.cellConnections & Directions::east) && tempFusedData.gridCell.cellState & CellState::visited)
				{
					updateCertainty -= 0.25f;
				}
			}
			else if (tempFusedData.robotState.mapCoordinate.y < lastDifferentPosittion.y)
			{
				walls |= Directions::west;

				if (!(~tempFusedData.gridCell.cellConnections & Directions::west) && tempFusedData.gridCell.cellState & CellState::visited)
				{
					updateCertainty -= 0.25f;
				}
			}

			if (updateCertainty < 0.0f) updateCertainty = 0.0f;

			tempCell.cellConnections = (~walls) & CellConnections::directionMask;

			MazeMapping::setCurrentCell(tempCell, tempFusedData.gridCellCertainty, updateCertainty, tempFusedData.robotState.mapCoordinate);

			if (validDistSpeedSamples > 0)
			{
				distSensSpeedTrust = validDistSpeedSamples / 4.0f;
				distSensSpeed = (-tempDistSensSpeed / (float)(validDistSpeedSamples)) * JAFDSettings::SensorFusion::distSensSpeedIIRFactor + distSensSpeed * (1.0f - JAFDSettings::SensorFusion::distSensSpeedIIRFactor);	// Negative, because increasing distance means driving away; 
			}
			else
			{
				distSensSpeedTrust = 0.0f;
			}

			lastPosition = tempFusedData.robotState.mapCoordinate;
			fusedData.gridCell = tempCell;
			fusedData.gridCellCertainty = tempFusedData.gridCellCertainty;
			lastTime = now;
		}

		void setCertainRobotPosition(Vec3f pos, Vec3f rotation)
		{
			auto tempRobotState = fusedData.robotState;

			tempRobotState.rotation.y = rotation.y;
			tempRobotState.rotation.z = rotation.z;

			float headingOffset = tempRobotState.rotation.x - rotation.x;

			while (headingOffset < -M_PI) headingOffset += M_TWOPI;
			while (headingOffset > M_PI) headingOffset -= M_TWOPI;

			totalHeadingOff += headingOffset;

			tempRobotState.rotation.x -= headingOffset;

			tempRobotState.position = pos;

			tempRobotState.mapCoordinate.x = roundf(tempRobotState.position.x / JAFDSettings::Field::cellWidth);
			tempRobotState.mapCoordinate.y = roundf(tempRobotState.position.y / JAFDSettings::Field::cellWidth);
			tempRobotState.mapCoordinate.floor = 0;

			if (tempRobotState.rotation.x > 315.0f * DEG_TO_RAD || tempRobotState.rotation.x <= 45.0f * DEG_TO_RAD) tempRobotState.heading = AbsoluteDir::north;
			else if (tempRobotState.rotation.x > 45.0f * DEG_TO_RAD && tempRobotState.rotation.x <= 135.0f * DEG_TO_RAD) tempRobotState.heading = AbsoluteDir::west;
			else if (tempRobotState.rotation.x > 135.0f * DEG_TO_RAD && tempRobotState.rotation.x <= 225.0f * DEG_TO_RAD) tempRobotState.heading = AbsoluteDir::south;
			else tempRobotState.heading = AbsoluteDir::east;

			fusedData.robotState = tempRobotState;
		}

		const volatile FusedData& getFusedData()
		{
			return fusedData;
		}

		void updateSensors()
		{
			DistanceSensors::forceNewMeasurement();
			
			if (ColorSensor::dataIsReady())
			{
				uint16_t colorTemp = 0;
				uint16_t lux = 0;
				ColorSensor::getData(&colorTemp, &lux);

				fusedData.colorSensData.colorTemp = colorTemp;
				fusedData.colorSensData.lux = lux;
			}

			Bno055::update_sensorreadings();

			RobotLogic::timeBetweenUpdate();
			
			DistanceSensors::updateDistSensors();
		}

		void setDistances(Distances distances)
		{
			fusedData.distances = distances;
		}

		void setDistSensStates(DistSensorStates distSensorStates)
		{
			fusedData.distSensorState = distSensorStates;
		}
	}
}