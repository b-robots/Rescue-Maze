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

#include <cmath>
#include <limits>

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
			volatile bool trustWheels = false;			// Should I trust the wheel measurements? Or are they slipping?
			volatile float distSensX = 0.0f;
			volatile float distSensY = 0.0f;
			volatile float distSensXTrust = 0.0f;
			volatile float distSensYTrust = 0.0f;
		}

		void sensorFiltering(const uint8_t freq)
		{
			RobotState tempRobotState = fusedData.robotState;

			tempRobotState.wheelSpeeds = MotorControl::getFloatSpeeds();

			// Magic factor: *1.173

			// Rotation
			auto lastHeading = tempRobotState.globalHeading;
			auto lastPitch = tempRobotState.pitch;
			float currHeading = 0.0f;	// We don't handle rotation of robot on ramp (pitch != 0°) completely correct! But it shouldn't matter.

			auto bnoForwardVec = Bno055::getForwardVec();
			auto bnoHeading = getGlobalHeading(bnoForwardVec);

			auto excpectedHeading = lastHeading + tempRobotState.angularVel.x / freq;
			bool bnoErr = false;

			if (Bno055::getRotSpeed() * DEG_TO_RAD > JAFDSettings::MotorControl::maxRotSpeed * 1.5f)
			{
				bnoHeading = fitAngleToInterval(excpectedHeading);
				bnoErr = true;
			}

			if (trustWheels)
			{
				currHeading = (MotorControl::getDistance(Motor::right) - MotorControl::getDistance(Motor::left)) / (JAFDSettings::Mechanics::wheelDistToMiddle * 2.0f * 1.173f);
				currHeading -= totalHeadingOff;
				currHeading = interpolateAngle(fitAngleToInterval(currHeading), bnoHeading, JAFDSettings::SensorFusion::bno055RotPortion);
			}
			else
			{
				currHeading = bnoHeading;
			}

			currHeading = interpolateAngle(currHeading, fitAngleToInterval(distSensAngle), distSensAngleTrust * JAFDSettings::SensorFusion::distAngularPortion);

			if (bnoErr)
			{
				tempRobotState.pitch += tempRobotState.angularVel.z * 0.5f;
			}
			else
			{
				tempRobotState.pitch = getPitch(bnoForwardVec) * JAFDSettings::SensorFusion::pitchIIRFactor + tempRobotState.pitch * (1.0f - JAFDSettings::SensorFusion::pitchIIRFactor);
			}

			tempRobotState.globalHeading = makeRotationCoherent(lastHeading, currHeading);

			tempRobotState.forwardVec = toForwardVec(tempRobotState.globalHeading, tempRobotState.pitch);		// Calculate forward vector

			// Angular velocity
			float encoderYawVel = (tempRobotState.wheelSpeeds.right - tempRobotState.wheelSpeeds.left) / (JAFDSettings::Mechanics::wheelDistToMiddle * 2.0f * 1.173f);

			auto lastAngularVel = tempRobotState.angularVel;

			tempRobotState.angularVel.x = encoderYawVel * (1.0f - JAFDSettings::SensorFusion::angularVelDiffPortion) + ((tempRobotState.globalHeading - lastHeading) / freq) * JAFDSettings::SensorFusion::angularVelDiffPortion;
			tempRobotState.angularVel.z = (tempRobotState.pitch - lastPitch) / freq;
			tempRobotState.angularVel.y = 0.0f;

			tempRobotState.angularVel = tempRobotState.angularVel * JAFDSettings::SensorFusion::angularVelIIRFactor + lastAngularVel * (1.0f - JAFDSettings::SensorFusion::angularVelIIRFactor);

			// Linear velocitys
			tempRobotState.forwardVel = ((tempRobotState.wheelSpeeds.left + tempRobotState.wheelSpeeds.right) / 2.0f) * (1.0f - distSensSpeedTrust * JAFDSettings::SensorFusion::distSpeedPortion) + distSensSpeed * (distSensSpeedTrust * JAFDSettings::SensorFusion::distSpeedPortion);

			tempRobotState.forwardVel = distSensSpeed * (distSensSpeedTrust * JAFDSettings::SensorFusion::distSpeedPortion) + tempRobotState.forwardVel * (1.0f - distSensSpeedTrust * JAFDSettings::SensorFusion::distSpeedPortion);

			// Position
			tempRobotState.position += tempRobotState.forwardVec * (tempRobotState.forwardVel / (float)freq);

			tempRobotState.position.x = distSensX * (JAFDSettings::SensorFusion::distSensOffsetPortion * distSensXTrust) + tempRobotState.position.x * (1.0f - JAFDSettings::SensorFusion::distSensOffsetPortion * distSensXTrust);
			tempRobotState.position.y = distSensY * (JAFDSettings::SensorFusion::distSensOffsetPortion * distSensYTrust) + tempRobotState.position.y * (1.0f - JAFDSettings::SensorFusion::distSensOffsetPortion * distSensYTrust);

			// Map coordinates
			tempRobotState.mapCoordinate.x = roundf(tempRobotState.position.x / JAFDSettings::Field::cellWidth);
			tempRobotState.mapCoordinate.y = roundf(tempRobotState.position.y / JAFDSettings::Field::cellWidth);

			// Heading
			float positiveAngle = tempRobotState.globalHeading;

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
			float tempDistSensSpeed = 0.0f;				// Measured speed 

			// Angle measurement with distances
			float tempDistSensAngle = 0.0f;				// Angle measured by distance sensors
			float tempDistSensAngleTrust = 0.0f;		// How much can I trust the measured angle? (0.0 - 1.0)

			// Wall detection (only for current segment)
			uint8_t frontWallsDetected = 0;		// How many times did a wall in front of us get detected
			uint8_t leftWallsDetected = 0;		// How many times did a wall left of us get detected
			uint8_t rightWallsDetected = 0;		// How many times did a wall right of us get detected

			// Border detection (wall with any offset front/back or length)
			uint8_t leftBorderDetected = 0;		// How many times did a border left of us get detected
			uint8_t rightBorderDetected = 0;		// How many times did a border right of us get detected

			struct
			{
				bool lf = false;
				bool lb = false;
				bool rf = false;
				bool rb = false;
			} borderDetected;

			// Offset calculation
			float tempXOffset = 0.0f;
			float tempYOffset = 0.0f;
			float tempXOffTrust = 0.0f;
			float tempYOffTrust = 0.0f;

			// MazeMapping
			static MapCoordinate lastDifferentPosittion = homePosition;
			static MapCoordinate lastPosition = homePosition;
			GridCell tempCell;
			float updateCertainty = 1.0f;
			tempCell.cellState = CellState::visited;
			tempCell.cellConnections = EntranceDirections::nowhere;
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

			if (fabsf(tempFusedData.robotState.pitch) < JAFDSettings::SensorFusion::maxPitchForDistSensor)
			{
				float headingCos = cosf(tempFusedData.robotState.globalHeading);
				float headingSin = sinf(tempFusedData.robotState.globalHeading);

				if (tempFusedData.distSensorState.frontLeft == DistSensorStatus::ok)
				{
					bool hitPointIsOk = false;

					// Measurement is ok
					// Check if resulting hit point is a 90° wall in front of us
					if (tempFusedData.robotState.heading == AbsoluteDir::north || tempFusedData.robotState.heading == AbsoluteDir::south)
					{
						float hitY = headingSin * tempFusedData.distances.frontLeft / 10.0f + tempFusedData.robotState.position.y + sinf(tempFusedData.robotState.globalHeading + JAFDSettings::Mechanics::distSensFrontAngleToMiddle) * JAFDSettings::Mechanics::distSensFrontDistToMiddle;

						if (fabsf(hitY - tempFusedData.robotState.mapCoordinate.y * JAFDSettings::Field::cellWidth) < JAFDSettings::MazeMapping::widthSecureDetectFactor * JAFDSettings::Field::cellWidth / 2.0f)
						{
							hitPointIsOk = true;

							// Helper variables for trig-calculations
							float cos1 = headingCos * tempFusedData.distances.frontLeft / 10.0f;
							float cos2 = cosf(tempFusedData.robotState.globalHeading + JAFDSettings::Mechanics::distSensFrontAngleToMiddle) * JAFDSettings::Mechanics::distSensFrontDistToMiddle;

							float hitX = cos1 + tempFusedData.robotState.position.x + cos2;

							if (fabsf(hitX - tempFusedData.robotState.mapCoordinate.x * JAFDSettings::Field::cellWidth) < JAFDSettings::Field::cellWidth / 2.0f + JAFDSettings::MazeMapping::distLongerThanBorder)
							{
								// Wall is directly in front of us
								frontWallsDetected++;

								// Cell-Midpoint offset calculation
								tempXOffset += JAFDSettings::Field::cellWidth / 2.0f * sgn(cos1) - cos1 - cos2;
								tempXOffTrust += 1.0f;
							}
						}
					}
					else
					{
						float hitX = headingCos * tempFusedData.distances.frontLeft / 10.0f + tempFusedData.robotState.position.x + cosf(tempFusedData.robotState.globalHeading + JAFDSettings::Mechanics::distSensFrontAngleToMiddle) * JAFDSettings::Mechanics::distSensFrontDistToMiddle;

						if (fabsf(hitX - tempFusedData.robotState.mapCoordinate.x * JAFDSettings::Field::cellWidth) < JAFDSettings::MazeMapping::widthSecureDetectFactor * JAFDSettings::Field::cellWidth / 2.0f)
						{
							hitPointIsOk = true;

							// Helper variables for trig-calculations
							float sin1 = headingSin * tempFusedData.distances.frontLeft / 10.0f;
							float sin2 = sinf(tempFusedData.robotState.globalHeading + JAFDSettings::Mechanics::distSensFrontAngleToMiddle) * JAFDSettings::Mechanics::distSensFrontDistToMiddle;

							float hitY = sin1 + tempFusedData.robotState.position.y + sin2;

							if (fabsf(hitY - tempFusedData.robotState.mapCoordinate.y * JAFDSettings::Field::cellWidth) < JAFDSettings::Field::cellWidth / 2.0f + JAFDSettings::MazeMapping::distLongerThanBorder)
							{
								// Wall is directly in front of us
								frontWallsDetected++;

								// Cell-Midpoint offset calculation
								tempYOffset += JAFDSettings::Field::cellWidth / 2.0f * sgn(sin1) - sin1 - sin2;
								tempYOffTrust += 1.0f;
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

						switch (tempFusedData.robotState.heading)
						{
						case AbsoluteDir::north:
							tempXOffset += 15.0f - JAFDSettings::Mechanics::robotLength / 2.0f;
							tempXOffTrust += 1.0f;
							break;
						case AbsoluteDir::east:
							tempYOffset += JAFDSettings::Mechanics::robotLength / 2.0f - 15.0f;
							tempYOffTrust += 1.0f;
							break;
						case AbsoluteDir::south:
							tempXOffset += JAFDSettings::Mechanics::robotLength / 2.0f - 15.0f;
							tempXOffTrust += 1.0f;
							break;
						case AbsoluteDir::west:
							tempYOffset += 15.0f - JAFDSettings::Mechanics::robotLength / 2.0f;
							tempYOffTrust += 1.0f;
							break;
						default:
							break;
						}
					}
				}

				if (tempFusedData.distSensorState.frontRight == DistSensorStatus::ok)
				{
					bool hitPointIsOk = false;

					// Measurement is ok
					// Check if resulting hit point is a 90° wall in front of us
					if (tempFusedData.robotState.heading == AbsoluteDir::north || tempFusedData.robotState.heading == AbsoluteDir::south)
					{
						float hitY = headingSin * tempFusedData.distances.frontRight / 10.0f + tempFusedData.robotState.position.y + sinf(tempFusedData.robotState.globalHeading - JAFDSettings::Mechanics::distSensFrontAngleToMiddle) * JAFDSettings::Mechanics::distSensFrontDistToMiddle;

						if (fabsf(hitY - tempFusedData.robotState.mapCoordinate.y * JAFDSettings::Field::cellWidth) < JAFDSettings::MazeMapping::widthSecureDetectFactor * JAFDSettings::Field::cellWidth / 2.0f)
						{
							hitPointIsOk = true;

							// Helper variables for trig-calculations
							float cos1 = headingCos * tempFusedData.distances.frontRight / 10.0f;
							float cos2 = cosf(tempFusedData.robotState.globalHeading - JAFDSettings::Mechanics::distSensFrontAngleToMiddle) * JAFDSettings::Mechanics::distSensFrontDistToMiddle;

							float hitX = cos1 + tempFusedData.robotState.position.x + cos2;

							if (fabsf(hitX - tempFusedData.robotState.mapCoordinate.x * JAFDSettings::Field::cellWidth) < JAFDSettings::Field::cellWidth / 2.0f + JAFDSettings::MazeMapping::distLongerThanBorder)
							{
								// Wall is directly in front of us
								frontWallsDetected++;

								// Cell-Midpoint offset calculation
								tempXOffset += JAFDSettings::Field::cellWidth / 2.0f * sgn(cos1) - cos1 - cos2;
								tempXOffTrust += 1.0f;
							}
						}
					}
					else
					{
						float hitX = headingCos * tempFusedData.distances.frontRight / 10.0f + tempFusedData.robotState.position.x + cosf(tempFusedData.robotState.globalHeading - JAFDSettings::Mechanics::distSensFrontAngleToMiddle) * JAFDSettings::Mechanics::distSensFrontDistToMiddle;

						if (fabsf(hitX - tempFusedData.robotState.mapCoordinate.x * JAFDSettings::Field::cellWidth) < JAFDSettings::MazeMapping::widthSecureDetectFactor * JAFDSettings::Field::cellWidth / 2.0f)
						{
							hitPointIsOk = true;

							// Helper variables for trig-calculation
							float sin1 = headingSin * tempFusedData.distances.frontRight / 10.0f;
							float sin2 = sinf(tempFusedData.robotState.globalHeading - JAFDSettings::Mechanics::distSensFrontAngleToMiddle) * JAFDSettings::Mechanics::distSensFrontDistToMiddle;

							float hitY = sin1 + tempFusedData.robotState.position.y + sin2;

							if (fabsf(hitY - tempFusedData.robotState.mapCoordinate.y * JAFDSettings::Field::cellWidth) < JAFDSettings::Field::cellWidth / 2.0f + JAFDSettings::MazeMapping::distLongerThanBorder)
							{
								// Wall is directly in front of us
								frontWallsDetected++;

								// Cell-Midpoint offset calculation
								tempYOffset += JAFDSettings::Field::cellWidth / 2.0f * sgn(sin1) - sin1 - sin2;
								tempYOffTrust += 1.0f;
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

						switch (tempFusedData.robotState.heading)
						{
						case AbsoluteDir::north:
							tempXOffset += 15.0f - JAFDSettings::Mechanics::robotLength / 2.0f;
							tempXOffTrust += 1.0f;
							break;
						case AbsoluteDir::east:
							tempYOffset += JAFDSettings::Mechanics::robotLength / 2.0f - 15.0f;
							tempYOffTrust += 1.0f;
							break;
						case AbsoluteDir::south:
							tempXOffset += JAFDSettings::Mechanics::robotLength / 2.0f - 15.0f;
							tempXOffTrust += 1.0f;
							break;
						case AbsoluteDir::west:
							tempYOffset += 15.0f - JAFDSettings::Mechanics::robotLength / 2.0f;
							tempYOffTrust += 1.0f;
							break;
						default:
							break;
						}
					}
				}

				if (tempFusedData.distSensorState.leftFront == DistSensorStatus::ok)
				{
					// Measurement is ok
					// Check if resulting hit point is a 90° wall in front of us
					if (tempFusedData.robotState.heading == AbsoluteDir::north || tempFusedData.robotState.heading == AbsoluteDir::south)
					{
						float cos1 = headingCos * tempFusedData.distances.leftFront / 10.0f;
						float cos2 = cosf(tempFusedData.robotState.globalHeading - JAFDSettings::Mechanics::distSensLRAngleToMiddle) * JAFDSettings::Mechanics::distSensLRDistToMiddle;

						float hitY = cos1 + tempFusedData.robotState.position.y + cos2;

						if (fabsf(hitY - tempFusedData.robotState.mapCoordinate.y * JAFDSettings::Field::cellWidth) < JAFDSettings::Field::cellWidth / 2.0f + JAFDSettings::MazeMapping::distLongerThanBorder)
						{
							leftBorderDetected++; borderDetected.lf = true;

							float hitX = -headingSin * tempFusedData.distances.leftFront / 10.0f + tempFusedData.robotState.position.x - sinf(tempFusedData.robotState.globalHeading - JAFDSettings::Mechanics::distSensLRAngleToMiddle) * JAFDSettings::Mechanics::distSensLRDistToMiddle;

							if (fabsf(hitX - tempFusedData.robotState.mapCoordinate.x * JAFDSettings::Field::cellWidth) < JAFDSettings::MazeMapping::widthSecureDetectFactor * JAFDSettings::Field::cellWidth / 2.0f)
							{
								// Wall is directly left of us
								leftWallsDetected++;

								tempYOffset += JAFDSettings::Field::cellWidth / 2.0f * sgn(cos1) - cos1 - cos2;
								tempYOffTrust += 1.0f;
							}
						}
					}
					else
					{
						float sin1 = -headingSin * tempFusedData.distances.leftFront / 10.0f;
						float sin2 = -sinf(tempFusedData.robotState.globalHeading + JAFDSettings::Mechanics::distSensLRAngleToMiddle) * JAFDSettings::Mechanics::distSensLRDistToMiddle;

						float hitX = sin1 + tempFusedData.robotState.position.x + sin2;

						if (fabsf(hitX - tempFusedData.robotState.mapCoordinate.x * JAFDSettings::Field::cellWidth) < JAFDSettings::Field::cellWidth / 2.0f + JAFDSettings::MazeMapping::distLongerThanBorder)
						{
							leftBorderDetected++; borderDetected.lf = true;

							float hitY = headingCos * tempFusedData.distances.leftFront / 10.0f + tempFusedData.robotState.position.y + cosf(tempFusedData.robotState.globalHeading + JAFDSettings::Mechanics::distSensLRAngleToMiddle) * JAFDSettings::Mechanics::distSensLRDistToMiddle;

							if (fabsf(hitY - tempFusedData.robotState.mapCoordinate.y * JAFDSettings::Field::cellWidth) < JAFDSettings::MazeMapping::widthSecureDetectFactor * JAFDSettings::Field::cellWidth / 2.0f)
							{
								// Wall is directly left of us
								leftWallsDetected++;

								tempXOffset += JAFDSettings::Field::cellWidth / 2.0f * sgn(sin1) - sin1 - sin2;
								tempXOffTrust += 1.0;
							}
						}
					}
				}
				else if (tempFusedData.distSensorState.leftFront == DistSensorStatus::underflow)
				{
					leftWallsDetected++;

					switch (tempFusedData.robotState.heading)
					{
					case AbsoluteDir::north:
						tempYOffset += 15.0f - JAFDSettings::Mechanics::robotWidth / 2.0f;
						tempYOffTrust += 1.0f;
						break;
					case AbsoluteDir::east:
						tempXOffset += JAFDSettings::Mechanics::robotWidth / 2.0f - 15.0f;
						tempXOffTrust += 1.0f;
						break;
					case AbsoluteDir::south:
						tempYOffset += JAFDSettings::Mechanics::robotWidth / 2.0f - 15.0f;
						tempYOffTrust += 1.0f;
						break;
					case AbsoluteDir::west:
						tempXOffset += 15.0f - JAFDSettings::Mechanics::robotWidth / 2.0f;
						tempXOffTrust += 1.0f;
						break;
					default:
						break;
					}
				}

				if (tempFusedData.distSensorState.leftBack == DistSensorStatus::ok)
				{
					// Measurement is ok
					// Check if resulting hit point is a 90° wall left of us
					if (tempFusedData.robotState.heading == AbsoluteDir::north || tempFusedData.robotState.heading == AbsoluteDir::south)
					{
						float cos1 = headingCos * tempFusedData.distances.leftBack / 10.0f;
						float cos2 = cosf(tempFusedData.robotState.globalHeading + JAFDSettings::Mechanics::distSensLRAngleToMiddle) * JAFDSettings::Mechanics::distSensLRDistToMiddle;

						float hitY = cos1 + tempFusedData.robotState.position.y + cos2;

						if (fabsf(hitY - tempFusedData.robotState.mapCoordinate.y * JAFDSettings::Field::cellWidth) < JAFDSettings::Field::cellWidth / 2.0f + JAFDSettings::MazeMapping::distLongerThanBorder)
						{
							leftBorderDetected++; borderDetected.lb = true;

							float hitX = -headingSin * tempFusedData.distances.leftBack / 10.0f + tempFusedData.robotState.position.x - sinf(tempFusedData.robotState.globalHeading + JAFDSettings::Mechanics::distSensLRAngleToMiddle) * JAFDSettings::Mechanics::distSensLRDistToMiddle;

							if (fabsf(hitX - tempFusedData.robotState.mapCoordinate.x * JAFDSettings::Field::cellWidth) < JAFDSettings::MazeMapping::widthSecureDetectFactor * JAFDSettings::Field::cellWidth / 2.0f)
							{
								// Wall is directly left of us
								leftWallsDetected++;

								tempYOffset += JAFDSettings::Field::cellWidth / 2.0f * sgn(cos1) - cos1 - cos2;
								tempYOffTrust += 1.0f;
							}
						}
					}
					else
					{
						float sin1 = -headingSin * tempFusedData.distances.leftBack / 10.0f;
						float sin2 = -sinf(tempFusedData.robotState.globalHeading + JAFDSettings::Mechanics::distSensLRAngleToMiddle) * JAFDSettings::Mechanics::distSensLRDistToMiddle;

						float hitX = sin1 + tempFusedData.robotState.position.x + sin2;

						if (fabsf(hitX - tempFusedData.robotState.mapCoordinate.x * JAFDSettings::Field::cellWidth) < JAFDSettings::Field::cellWidth / 2.0f + JAFDSettings::MazeMapping::distLongerThanBorder)
						{
							leftBorderDetected++; borderDetected.lb = true;

							float hitY = headingCos * tempFusedData.distances.leftBack / 10.0f + tempFusedData.robotState.position.y + cosf(tempFusedData.robotState.globalHeading + JAFDSettings::Mechanics::distSensLRAngleToMiddle) * JAFDSettings::Mechanics::distSensLRDistToMiddle;

							if (fabsf(hitY - tempFusedData.robotState.mapCoordinate.y * JAFDSettings::Field::cellWidth) < JAFDSettings::MazeMapping::widthSecureDetectFactor * JAFDSettings::Field::cellWidth / 2.0f)
							{
								// Wall is directly left of us
								leftWallsDetected++;

								tempXOffset += JAFDSettings::Field::cellWidth / 2.0f * sgn(sin1) - sin1 - sin2;
								tempXOffTrust += 1.0f;
							}
						}
					}
				}
				else if (tempFusedData.distSensorState.leftBack == DistSensorStatus::underflow)
				{
					leftWallsDetected++;

					switch (tempFusedData.robotState.heading)
					{
					case AbsoluteDir::north:
						tempYOffset += 15.0f - JAFDSettings::Mechanics::robotWidth / 2.0f;
						tempYOffTrust += 1.0f;
						break;
					case AbsoluteDir::east:
						tempXOffset += JAFDSettings::Mechanics::robotWidth / 2.0f - 15.0f;
						tempXOffTrust += 1.0f;
						break;
					case AbsoluteDir::south:
						tempYOffset += JAFDSettings::Mechanics::robotWidth / 2.0f - 15.0f;
						tempYOffTrust += 1.0f;
						break;
					case AbsoluteDir::west:
						tempXOffset += 15.0f - JAFDSettings::Mechanics::robotWidth / 2.0f;
						tempXOffTrust += 1.0f;
						break;
					default:
						break;
					}
				}

				if (tempFusedData.distSensorState.rightFront == DistSensorStatus::ok)
				{
					// Measurement is ok
					// Check if resulting hit point is a 90° wall right of us
					if (tempFusedData.robotState.heading == AbsoluteDir::north || tempFusedData.robotState.heading == AbsoluteDir::south)
					{
						float cos1 = -headingCos * tempFusedData.distances.rightFront / 10.0f;
						float cos2 = -cosf(tempFusedData.robotState.globalHeading + JAFDSettings::Mechanics::distSensLRAngleToMiddle) * JAFDSettings::Mechanics::distSensLRDistToMiddle;

						float hitY = cos1 + tempFusedData.robotState.position.y + cos2;

						if (fabsf(hitY - tempFusedData.robotState.mapCoordinate.y * JAFDSettings::Field::cellWidth) < JAFDSettings::Field::cellWidth / 2.0f + JAFDSettings::MazeMapping::distLongerThanBorder)
						{
							rightBorderDetected++; borderDetected.rf = true;

							float hitX = headingSin * tempFusedData.distances.rightFront / 10.0f + tempFusedData.robotState.position.x + sinf(tempFusedData.robotState.globalHeading + JAFDSettings::Mechanics::distSensLRAngleToMiddle) * JAFDSettings::Mechanics::distSensLRDistToMiddle;

							if (fabsf(hitX - tempFusedData.robotState.mapCoordinate.x * JAFDSettings::Field::cellWidth) < JAFDSettings::MazeMapping::widthSecureDetectFactor * JAFDSettings::Field::cellWidth / 2.0f)
							{
								// Wall is directly right of us
								rightWallsDetected++;

								tempYOffset += JAFDSettings::Field::cellWidth / 2.0f * sgn(cos1) - cos1 - cos2;
								tempYOffTrust += 1.0f;
							}
						}
					}
					else
					{
						float sin1 = headingSin * tempFusedData.distances.rightFront / 10.0f;
						float sin2 = sinf(tempFusedData.robotState.globalHeading + JAFDSettings::Mechanics::distSensLRAngleToMiddle) * JAFDSettings::Mechanics::distSensLRDistToMiddle;

						float hitX = sin1 + tempFusedData.robotState.position.x + sin2;

						if (fabsf(hitX - tempFusedData.robotState.mapCoordinate.x * JAFDSettings::Field::cellWidth) < JAFDSettings::Field::cellWidth / 2.0f + JAFDSettings::MazeMapping::distLongerThanBorder)
						{
							rightBorderDetected++; borderDetected.rf = true;

							float hitY = -headingCos * tempFusedData.distances.rightFront / 10.0f + tempFusedData.robotState.position.y - cosf(tempFusedData.robotState.globalHeading + JAFDSettings::Mechanics::distSensLRAngleToMiddle) * JAFDSettings::Mechanics::distSensLRDistToMiddle;

							if (fabsf(hitY - tempFusedData.robotState.mapCoordinate.y * JAFDSettings::Field::cellWidth) < JAFDSettings::MazeMapping::widthSecureDetectFactor * JAFDSettings::Field::cellWidth / 2.0f)
							{
								// Wall is directly right of us
								rightWallsDetected++;

								tempXOffset += JAFDSettings::Field::cellWidth / 2.0f * sgn(sin1) - sin1 - sin2;
								tempXOffTrust += 1.0f;
							}
						}
					}
				}
				else if (tempFusedData.distSensorState.rightFront == DistSensorStatus::underflow)
				{
					rightWallsDetected++;

					switch (tempFusedData.robotState.heading)
					{
					case AbsoluteDir::south:
						tempYOffset += 15.0f - JAFDSettings::Mechanics::robotWidth / 2.0f;
						tempYOffTrust += 1.0f;
						break;
					case AbsoluteDir::west:
						tempXOffset += JAFDSettings::Mechanics::robotWidth / 2.0f - 15.0f;
						tempXOffTrust += 1.0f;
						break;
					case AbsoluteDir::north:
						tempYOffset += JAFDSettings::Mechanics::robotWidth / 2.0f - 15.0f;
						tempYOffTrust += 1.0f;
						break;
					case AbsoluteDir::east:
						tempXOffset += 15.0f - JAFDSettings::Mechanics::robotWidth / 2.0f;
						tempXOffTrust += 1.0f;
						break;
					default:
						break;
					}
				}

				if (tempFusedData.distSensorState.rightBack == DistSensorStatus::ok)
				{
					// Measurement is ok
					// Check if resulting hit point is a 90° wall right of us
					if (tempFusedData.robotState.heading == AbsoluteDir::north || tempFusedData.robotState.heading == AbsoluteDir::south)
					{
						float cos1 = -headingCos * tempFusedData.distances.rightBack / 10.0f;
						float cos2 = -cosf(tempFusedData.robotState.globalHeading - JAFDSettings::Mechanics::distSensLRAngleToMiddle) * JAFDSettings::Mechanics::distSensLRDistToMiddle;

						float hitY = cos1 + tempFusedData.robotState.position.y + cos2;

						if (fabsf(hitY - tempFusedData.robotState.mapCoordinate.y * JAFDSettings::Field::cellWidth) < JAFDSettings::Field::cellWidth / 2.0f + JAFDSettings::MazeMapping::distLongerThanBorder)
						{
							rightBorderDetected++; borderDetected.rb = true;

							float hitX = headingSin * tempFusedData.distances.rightBack / 10.0f + tempFusedData.robotState.position.x + sinf(tempFusedData.robotState.globalHeading - JAFDSettings::Mechanics::distSensLRAngleToMiddle) * JAFDSettings::Mechanics::distSensLRDistToMiddle;

							if (fabsf(hitX - tempFusedData.robotState.mapCoordinate.x * JAFDSettings::Field::cellWidth) < JAFDSettings::MazeMapping::widthSecureDetectFactor * JAFDSettings::Field::cellWidth / 2.0f)
							{
								// Wall is directly right of us
								rightWallsDetected++;

								tempYOffset += JAFDSettings::Field::cellWidth / 2.0f * sgn(cos1) - cos1 - cos2;
								tempYOffTrust += 1.0f;
							}
						}
					}
					else
					{
						float sin1 = headingSin * tempFusedData.distances.rightBack / 10.0f;
						float sin2 = sinf(tempFusedData.robotState.globalHeading - JAFDSettings::Mechanics::distSensLRAngleToMiddle) * JAFDSettings::Mechanics::distSensLRDistToMiddle;

						float hitX = sin1 + tempFusedData.robotState.position.x + sin2;
						if (fabsf(hitX - tempFusedData.robotState.mapCoordinate.x * JAFDSettings::Field::cellWidth) < JAFDSettings::Field::cellWidth / 2.0f + JAFDSettings::MazeMapping::distLongerThanBorder)
						{
							rightBorderDetected++; borderDetected.rb = true;

							float hitY = -headingCos * tempFusedData.distances.rightBack / 10.0f + tempFusedData.robotState.position.y - cosf(tempFusedData.robotState.globalHeading - JAFDSettings::Mechanics::distSensLRAngleToMiddle) * JAFDSettings::Mechanics::distSensLRDistToMiddle;

							if (fabsf(hitY - tempFusedData.robotState.mapCoordinate.y * JAFDSettings::Field::cellWidth) < JAFDSettings::MazeMapping::widthSecureDetectFactor * JAFDSettings::Field::cellWidth / 2.0f)
							{
								// Wall is directly right of us
								rightWallsDetected++;

								tempXOffset += JAFDSettings::Field::cellWidth / 2.0f * sgn(sin1) - sin1 - sin2;
								tempXOffTrust += 1.0f;
							}
						}
					}
				}
				else if (tempFusedData.distSensorState.rightBack == DistSensorStatus::underflow)
				{
					rightWallsDetected++;

					switch (tempFusedData.robotState.heading)
					{
					case AbsoluteDir::south:
						tempYOffset += 15.0f - JAFDSettings::Mechanics::robotWidth / 2.0f;
						tempYOffTrust += 1.0f;
						break;
					case AbsoluteDir::west:
						tempXOffset += JAFDSettings::Mechanics::robotWidth / 2.0f - 15.0f;
						tempXOffTrust += 1.0f;
						break;
					case AbsoluteDir::north:
						tempYOffset += JAFDSettings::Mechanics::robotWidth / 2.0f - 15.0f;
						tempYOffTrust += 1.0f;
						break;
					case AbsoluteDir::east:
						tempXOffset += 15.0f - JAFDSettings::Mechanics::robotWidth / 2.0f;
						tempXOffTrust += 1.0f;
						break;
					default:
						break;
					}
				}

				// Calculate angle
				if (frontWallsDetected == 2 && tempFusedData.distSensorState.frontLeft == DistSensorStatus::ok && tempFusedData.distSensorState.frontRight == DistSensorStatus::ok)
				{
					// Calculate angle if both front distance sensors detected a wall directly in front of the robot.
					tempDistSensAngle += asinf((tempFusedData.distances.frontLeft - tempFusedData.distances.frontRight) / sqrtf(JAFDSettings::Mechanics::distSensFrontSpacing * JAFDSettings::Mechanics::distSensFrontSpacing * 100.0f + (tempFusedData.distances.frontLeft - tempFusedData.distances.frontRight) * (tempFusedData.distances.frontLeft - tempFusedData.distances.frontRight)));
					tempDistSensAngleTrust += 1.0f / 3.0f;
				}

				if (leftBorderDetected == 2 && tempFusedData.distSensorState.leftFront == DistSensorStatus::ok && tempFusedData.distSensorState.leftBack == DistSensorStatus::ok)
				{
					// Calculate angle if both left distance sensors detected a border directly left of the robot.
					tempDistSensAngle += asinf((tempFusedData.distances.leftBack - tempFusedData.distances.leftFront) / sqrtf(JAFDSettings::Mechanics::distSensLRSpacing * JAFDSettings::Mechanics::distSensLRSpacing * 100.0f + (tempFusedData.distances.leftBack - tempFusedData.distances.leftFront) * (tempFusedData.distances.leftBack - tempFusedData.distances.leftFront)));
					tempDistSensAngleTrust += 1.0f / 3.0f;
				}

				if (rightBorderDetected == 2 && tempFusedData.distSensorState.rightFront == DistSensorStatus::ok && tempFusedData.distSensorState.rightBack == DistSensorStatus::ok)

				{
					// Calculate angle if both right distance sensors detected a wall directly right of the robot.
					tempDistSensAngle += asinf((tempFusedData.distances.rightFront - tempFusedData.distances.rightBack) / sqrtf(JAFDSettings::Mechanics::distSensLRSpacing * JAFDSettings::Mechanics::distSensLRSpacing * 100.0f + (tempFusedData.distances.rightFront - tempFusedData.distances.rightBack) * (tempFusedData.distances.rightFront - tempFusedData.distances.rightBack)));
					tempDistSensAngleTrust += 1.0f / 3.0f;
				}

				// !!! need to update factor of trust (new factor 3)

				if (tempDistSensAngleTrust > 0.01f)
				{
					tempDistSensAngle /= tempDistSensAngleTrust * 3.0f;
				}

				tempXOffset /= tempXOffTrust;
				tempYOffset /= tempYOffTrust;

				if (tempXOffTrust < 0.01f) tempXOffset = 0.0f;
				if (tempYOffTrust < 0.01f) tempYOffset = 0.0f;

				switch (tempFusedData.robotState.heading)
				{

				case AbsoluteDir::north:
					distSensAngle = tempDistSensAngle;
					tempXOffTrust /= 2.0f;		// If facing north, two sensors (front) could give an X-Offset
					tempYOffTrust /= 4.0f;		// If facing north, four sensors (left & right) could give an Y-Offset
					break;

				case AbsoluteDir::east:
					distSensAngle = tempDistSensAngle - DEG_TO_RAD * 90.0f;
					tempXOffTrust /= 4.0f;
					tempYOffTrust /= 2.0f;
					break;

				case AbsoluteDir::south:
					distSensAngle = tempDistSensAngle + DEG_TO_RAD * 180.0f;
					tempXOffTrust /= 2.0f;
					tempYOffTrust /= 4.0f;
					break;

				case AbsoluteDir::west:
					distSensAngle = tempDistSensAngle + DEG_TO_RAD * 90.0f;
					tempXOffTrust /= 4.0f;
					tempYOffTrust /= 2.0f;
					break;

				default:
					break;
				}

				distSensX = tempXOffset + tempFusedData.robotState.mapCoordinate.x * JAFDSettings::Field::cellWidth;
				distSensY = tempYOffset + tempFusedData.robotState.mapCoordinate.y * JAFDSettings::Field::cellWidth;
				distSensXTrust = tempXOffTrust;
				distSensYTrust = tempYOffTrust;

				distSensAngleTrust = tempDistSensAngleTrust;

				if (tempFusedData.distSensorState.frontLong == DistSensorStatus::ok)
				{
					bool hitPointIsOk = false;

					// Measurement is ok
					// Check if resulting hit point is a 90° wall in front of us
					if (tempFusedData.robotState.heading == AbsoluteDir::north || tempFusedData.robotState.heading == AbsoluteDir::south)
					{
						float hitY = headingSin * (tempFusedData.distances.frontLong / 10.0f + JAFDSettings::Mechanics::distSensFrontBackDist / 2.0f) + tempFusedData.robotState.position.y;

						if (fabsf(hitY - tempFusedData.robotState.mapCoordinate.y * JAFDSettings::Field::cellWidth) < JAFDSettings::MazeMapping::widthSecureDetectFactor * JAFDSettings::Field::cellWidth / 2.0f)
						{
							hitPointIsOk = true;
						}
					}
					else
					{
						float hitX = headingCos * (tempFusedData.distances.frontRight / 10.0f + JAFDSettings::Mechanics::distSensFrontBackDist / 2.0f) + tempFusedData.robotState.position.x;

						if (fabsf(hitX - tempFusedData.robotState.mapCoordinate.x * JAFDSettings::Field::cellWidth) < JAFDSettings::MazeMapping::widthSecureDetectFactor * JAFDSettings::Field::cellWidth / 2.0f)
						{
							hitPointIsOk = true;
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
			}

			if (frontWallsDetected > 0)
			{
				switch (makeAbsolute(RelativeDir::forward, tempFusedData.robotState.heading))
				{

				case AbsoluteDir::north:
				{
					if ((~tempFusedData.gridCell.cellConnections & EntranceDirections::north && tempFusedData.gridCell.cellState & CellState::visited) || frontWallsDetected > 1)
					{
						walls |= EntranceDirections::north;
					}
					else if (!(tempFusedData.gridCell.cellState & CellState::visited))
					{
						walls |= EntranceDirections::north;
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
					if ((~tempFusedData.gridCell.cellConnections & EntranceDirections::east && tempFusedData.gridCell.cellState & CellState::visited) || frontWallsDetected > 1)
					{
						walls |= EntranceDirections::east;
					}
					else if (!(tempFusedData.gridCell.cellState & CellState::visited))
					{
						walls |= EntranceDirections::east;
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
					if ((~tempFusedData.gridCell.cellConnections & EntranceDirections::south && tempFusedData.gridCell.cellState & CellState::visited) || frontWallsDetected > 1)
					{
						walls |= EntranceDirections::south;
					}
					else if (!(tempFusedData.gridCell.cellState & CellState::visited))
					{
						walls |= EntranceDirections::south;
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
					if ((~tempFusedData.gridCell.cellConnections & EntranceDirections::west && tempFusedData.gridCell.cellState & CellState::visited) || frontWallsDetected > 1)
					{
						walls |= EntranceDirections::west;
					}
					else if (!(tempFusedData.gridCell.cellState & CellState::visited))
					{
						walls |= EntranceDirections::west;
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
					if ((~tempFusedData.gridCell.cellConnections & EntranceDirections::north && tempFusedData.gridCell.cellState & CellState::visited) || leftWallsDetected > 1)
					{
						walls |= EntranceDirections::north;
					}
					else if (!(tempFusedData.gridCell.cellState & CellState::visited))
					{
						walls |= EntranceDirections::north;
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
					if ((~tempFusedData.gridCell.cellConnections & EntranceDirections::east && tempFusedData.gridCell.cellState & CellState::visited) || leftWallsDetected > 1)
					{
						walls |= EntranceDirections::east;
					}
					else if (!(tempFusedData.gridCell.cellState & CellState::visited))
					{
						walls |= EntranceDirections::east;
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
					if ((~tempFusedData.gridCell.cellConnections & EntranceDirections::south && tempFusedData.gridCell.cellState & CellState::visited) || leftWallsDetected > 1)
					{
						walls |= EntranceDirections::south;
					}
					else if (!(tempFusedData.gridCell.cellState & CellState::visited))
					{
						walls |= EntranceDirections::south;
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
					if ((~tempFusedData.gridCell.cellConnections & EntranceDirections::west && tempFusedData.gridCell.cellState & CellState::visited) || leftWallsDetected > 1)
					{
						walls |= EntranceDirections::west;
					}
					else if (!(tempFusedData.gridCell.cellState & CellState::visited))
					{
						walls |= EntranceDirections::west;
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
					if ((~tempFusedData.gridCell.cellConnections & EntranceDirections::north && tempFusedData.gridCell.cellState & CellState::visited) || rightWallsDetected > 1)
					{
						walls |= EntranceDirections::north;
					}
					else if (!(tempFusedData.gridCell.cellState & CellState::visited))
					{
						walls |= EntranceDirections::north;
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
					if ((~tempFusedData.gridCell.cellConnections & EntranceDirections::east && tempFusedData.gridCell.cellState & CellState::visited) || rightWallsDetected > 1)
					{
						walls |= EntranceDirections::east;
					}
					else if (!(tempFusedData.gridCell.cellState & CellState::visited))
					{
						walls |= EntranceDirections::east;
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
					if ((~tempFusedData.gridCell.cellConnections & EntranceDirections::south && tempFusedData.gridCell.cellState & CellState::visited) || rightWallsDetected > 1)
					{
						walls |= EntranceDirections::south;
					}
					else if (!(tempFusedData.gridCell.cellState & CellState::visited))
					{
						walls |= EntranceDirections::south;
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
					if ((~tempFusedData.gridCell.cellConnections & EntranceDirections::west && tempFusedData.gridCell.cellState & CellState::visited) || rightWallsDetected > 1)
					{
						walls |= EntranceDirections::west;
					}
					else if (!(tempFusedData.gridCell.cellState & CellState::visited))
					{
						walls |= EntranceDirections::west;
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
				walls |= EntranceDirections::south;

				if (!(~tempFusedData.gridCell.cellConnections & EntranceDirections::south) && tempFusedData.gridCell.cellState & CellState::visited)
				{
					updateCertainty -= 0.25f;
				}
			}
			else if (tempFusedData.robotState.mapCoordinate.x < lastDifferentPosittion.x)
			{
				walls |= EntranceDirections::north;

				if (!(~tempFusedData.gridCell.cellConnections & EntranceDirections::north) && tempFusedData.gridCell.cellState & CellState::visited)
				{
					updateCertainty -= 0.25f;
				}
			}

			if (tempFusedData.robotState.mapCoordinate.y > lastDifferentPosittion.y)
			{
				walls |= EntranceDirections::east;

				if (!(~tempFusedData.gridCell.cellConnections & EntranceDirections::east) && tempFusedData.gridCell.cellState & CellState::visited)
				{
					updateCertainty -= 0.25f;
				}
			}
			else if (tempFusedData.robotState.mapCoordinate.y < lastDifferentPosittion.y)
			{
				walls |= EntranceDirections::west;

				if (!(~tempFusedData.gridCell.cellConnections & EntranceDirections::west) && tempFusedData.gridCell.cellState & CellState::visited)
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

		// "heading" in rad
		void setCertainRobotPosition(Vec3f pos, float heading)
		{
			auto tempRobotState = fusedData.robotState;

			tempRobotState.position = pos;
			tempRobotState.globalHeading = makeRotationCoherent(tempRobotState.globalHeading, heading);

			float currentRotEncAngle = (MotorControl::getDistance(Motor::right) - MotorControl::getDistance(Motor::left)) / (JAFDSettings::Mechanics::wheelDistToMiddle * 2.0f * 1.173f);

			totalHeadingOff = fitAngleToInterval(heading - currentRotEncAngle);

			Bno055::tare(heading);

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

			Bno055::updateValues();

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