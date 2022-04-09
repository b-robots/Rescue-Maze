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
#include "../header/SmallThings.h"
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
			volatile bool trustWheels = true;			// Should I trust the wheel measurements? Or are they slipping?
			volatile bool isDrivingStraight = false;	// Is the robot driving straight / following a wall?

			volatile struct {
				float l = -1.0f;
				float r = -1.0f;
				float f = -1.0f;
			} distToWalls;

			volatile struct {
				bool zeroPitch = false;
				bool newX = false;
				bool newY = false;
				bool newHeading = false;
				float x;
				float y;
				float heading;
			} correctedState;
		}

		volatile bool bnoErr = false;

		void setDrivingStraight(bool _isDrivingStraight) {
			isDrivingStraight = _isDrivingStraight;
		}

		void sensorFiltering(const uint8_t freq)
		{
			if (!isDrivingStraight) {
				distSensAngleTrust = 0.0f;
				distSensSpeedTrust = 0.0f;
			}

			RobotState tempRobotState = fusedData.robotState;

			tempRobotState.wheelSpeeds = MotorControl::getFloatSpeeds();

			// Rotation
			// We don't handle rotation of robot on ramp (pitch != 0°) completely correct! But it shouldn't matter.
			static float lastBnoHeading = 0.0f;
			const auto lastHeading = tempRobotState.globalHeading;
			const auto lastAngularVel = tempRobotState.angularVel;
			const auto lastPitch = tempRobotState.pitch;
			float currHeading = 0.0f;

			const auto bnoForwardVec = Bno055::getForwardVec();
			auto bnoHeading = getGlobalHeading(bnoForwardVec);

			auto excpectedHeading = fitAngleToInterval(lastHeading + lastAngularVel.x / freq * 0.8);
			bnoErr = false;

			// too much angular change (max. 3 rad/s) -> BNO error
			if (fabsf(fitAngleToInterval(bnoHeading - lastBnoHeading)) > 3.0f / freq || Bno055::getOverallCalibStatus() < 1 || fabsf(fitAngleToInterval(getPitch(bnoForwardVec) - lastPitch)) > 3.0f / freq)
			{
				bnoErr = true;
			}

			if (bnoErr)
			{
				//tempRobotState.pitch += tempRobotState.angularVel.z / freq * 0.2f;
			}
			else
			{
				tempRobotState.pitch = getPitch(bnoForwardVec) * JAFDSettings::SensorFusion::pitchIIRFactor + lastPitch * (1.0f - JAFDSettings::SensorFusion::pitchIIRFactor);
			}

			tempRobotState.angularVel.z = (tempRobotState.pitch - lastPitch) * freq;

			float trustYawVel = 0.0f;

			if (trustWheels)
			{
				// Magic factor: *1.13
				float encoderYawVel = (tempRobotState.wheelSpeeds.right - tempRobotState.wheelSpeeds.left) / (JAFDSettings::Mechanics::wheelDistToMiddle * 2.0f * JAFDSettings::MotorControl::magicFactor);
				
				if (!bnoErr) {
					trustYawVel = 1.0f;
					tempRobotState.angularVel.x = fitAngleToInterval(bnoHeading - lastBnoHeading) * freq * JAFDSettings::SensorFusion::bno055DiffPortion + encoderYawVel * (1.0f - JAFDSettings::SensorFusion::bno055DiffPortion);
				}
				else {
					trustYawVel = 0.7f;
					tempRobotState.angularVel.x = encoderYawVel;
				}

				currHeading = (MotorControl::getDistance(Motor::right) - MotorControl::getDistance(Motor::left)) / (JAFDSettings::Mechanics::wheelDistToMiddle * 2.0f * JAFDSettings::MotorControl::magicFactor);
				currHeading -= totalHeadingOff;

				if (!bnoErr) {
					currHeading = interpolateAngle(fitAngleToInterval(currHeading), bnoHeading, JAFDSettings::SensorFusion::bno055RotPortion);
				}

				currHeading = interpolateAngle(currHeading, fitAngleToInterval(distSensAngle), distSensAngleTrust * JAFDSettings::SensorFusion::distAngularPortion);
			}
			else
			{
				if (!bnoErr) {
					trustYawVel = 0.6f;
					tempRobotState.angularVel.x = fitAngleToInterval(bnoHeading - lastBnoHeading) * freq;
					currHeading = interpolateAngle(bnoHeading, fitAngleToInterval(distSensAngle), sqrtf(distSensAngleTrust * JAFDSettings::SensorFusion::distAngularPortion * (1.0f - JAFDSettings::SensorFusion::bno055RotPortion)));
				}
				else if (distSensAngleTrust > 0.001f) {
					currHeading = fitAngleToInterval(distSensAngle);
				}
				else {
					currHeading = excpectedHeading;
				}
			}

			// mix asolute and relative heading
			currHeading = interpolateAngle(currHeading, fitAngleToInterval(lastHeading + tempRobotState.angularVel.x / freq), trustYawVel * JAFDSettings::SensorFusion::angleDiffPortion);

			if (trustYawVel < 0.001f) {
				tempRobotState.angularVel.x = fitAngleToInterval(currHeading - lastHeading) * freq;
			}

			tempRobotState.angularVel.y = 0.0f;

			tempRobotState.angularVel = tempRobotState.angularVel * JAFDSettings::SensorFusion::angularVelIIRFactor + lastAngularVel * (1.0f - JAFDSettings::SensorFusion::angularVelIIRFactor);

			if (correctedState.newHeading) {
				currHeading = correctedState.heading;
				correctedState.newHeading = false;
			}

			if (correctedState.zeroPitch) {
				tempRobotState.pitch = 0.0f;
				tempRobotState.angularVel.z = 0.0f;
				correctedState.zeroPitch = false;
			}

			tempRobotState.globalHeading = makeRotationCoherent(lastHeading, currHeading);

			tempRobotState.forwardVec = toForwardVec(tempRobotState.globalHeading, tempRobotState.pitch);		// Calculate forward vector

			// Linear velocitys
			tempRobotState.forwardVel = ((tempRobotState.wheelSpeeds.left + tempRobotState.wheelSpeeds.right) / 2.0f) * (1.0f - distSensSpeedTrust * JAFDSettings::SensorFusion::distSpeedPortion) + distSensSpeed * (distSensSpeedTrust * JAFDSettings::SensorFusion::distSpeedPortion);

			tempRobotState.forwardVel = distSensSpeed * (distSensSpeedTrust * JAFDSettings::SensorFusion::distSpeedPortion) + tempRobotState.forwardVel * (1.0f - distSensSpeedTrust * JAFDSettings::SensorFusion::distSpeedPortion);

			// Position
			tempRobotState.position += tempRobotState.forwardVec * (tempRobotState.forwardVel / (float)freq);

			if (correctedState.newX) {
				tempRobotState.position.x = correctedState.x;
				correctedState.newX = false;
			}

			if (correctedState.newY) {
				tempRobotState.position.y = correctedState.y;
				correctedState.newY = false;
			}

			// Map absolute heading & position (MAYBE just for DEBUGGING)
			if (currHeading > M_PI_4 && currHeading < M_3PI_4) {
				tempRobotState.heading = AbsoluteDir::west;
			}
			else if (currHeading < -M_PI_4 && currHeading > -M_3PI_4) {
				tempRobotState.heading = AbsoluteDir::east;
			}
			else if (currHeading < M_PI_4 && currHeading > -M_PI_4) {
				tempRobotState.heading = AbsoluteDir::north;
			}
			else {
				tempRobotState.heading = AbsoluteDir::south;
			}

			auto prevCoord = tempRobotState.mapCoordinate;

			tempRobotState.mapCoordinate.x = (int8_t)roundf(tempRobotState.position.x / JAFDSettings::Field::cellWidth);
			tempRobotState.mapCoordinate.y = (int8_t)roundf(tempRobotState.position.y / JAFDSettings::Field::cellWidth);

			if (tempRobotState.mapCoordinate != prevCoord) {
				fusedData.gridCell = GridCell();
			}

			fusedData.robotState = tempRobotState;

			lastBnoHeading = bnoHeading;
		}

		bool scanSurrounding() {
			uint8_t frontWallsDetected = 0;		// How many times did a wall in front of us get detected
			uint8_t leftWallsDetected = 0;		// How many times did a wall left of us get detected
			uint8_t rightWallsDetected = 0;		// How many times did a wall right of us get detected

			auto tempFusedData = fusedData;

			float headingCos = cosf(tempFusedData.robotState.globalHeading);
			float headingSin = sinf(tempFusedData.robotState.globalHeading);

			if (tempFusedData.distSensorState.frontLeft == DistSensorStatus::ok)
			{
				// Measurement is ok
				// Check if resulting hit point is a 90° wall in front of us
				if (tempFusedData.robotState.heading == AbsoluteDir::north || tempFusedData.robotState.heading == AbsoluteDir::south)
				{
					float hitY = headingSin * tempFusedData.distances.frontLeft / 10.0f + tempFusedData.robotState.position.y + sinf(tempFusedData.robotState.globalHeading + JAFDSettings::Mechanics::distSensFrontAngleToMiddle) * JAFDSettings::Mechanics::distSensFrontDistToMiddle;

					if (fabsf(hitY - tempFusedData.robotState.mapCoordinate.y * JAFDSettings::Field::cellWidth) < JAFDSettings::MazeMapping::widthSecureDetectFactor * JAFDSettings::Field::cellWidth / 2.0f)
					{
						// Helper variables for trig-calculations
						float cos1 = headingCos * tempFusedData.distances.frontLeft / 10.0f;
						float cos2 = cosf(tempFusedData.robotState.globalHeading + JAFDSettings::Mechanics::distSensFrontAngleToMiddle) * JAFDSettings::Mechanics::distSensFrontDistToMiddle;

						float hitX = cos1 + tempFusedData.robotState.position.x + cos2;

						if (fabsf(hitX - tempFusedData.robotState.mapCoordinate.x * JAFDSettings::Field::cellWidth) < JAFDSettings::Field::cellWidth / 2.0f + JAFDSettings::MazeMapping::distLongerThanBorder)
						{
							// Wall is directly in front of us
							frontWallsDetected++;
						}
					}
				}
				else
				{
					float hitX = headingCos * tempFusedData.distances.frontLeft / 10.0f + tempFusedData.robotState.position.x + cosf(tempFusedData.robotState.globalHeading + JAFDSettings::Mechanics::distSensFrontAngleToMiddle) * JAFDSettings::Mechanics::distSensFrontDistToMiddle;

					if (fabsf(hitX - tempFusedData.robotState.mapCoordinate.x * JAFDSettings::Field::cellWidth) < JAFDSettings::MazeMapping::widthSecureDetectFactor * JAFDSettings::Field::cellWidth / 2.0f)
					{
						// Helper variables for trig-calculations
						float sin1 = headingSin * tempFusedData.distances.frontLeft / 10.0f;
						float sin2 = sinf(tempFusedData.robotState.globalHeading + JAFDSettings::Mechanics::distSensFrontAngleToMiddle) * JAFDSettings::Mechanics::distSensFrontDistToMiddle;

						float hitY = sin1 + tempFusedData.robotState.position.y + sin2;

						if (fabsf(hitY - tempFusedData.robotState.mapCoordinate.y * JAFDSettings::Field::cellWidth) < JAFDSettings::Field::cellWidth / 2.0f + JAFDSettings::MazeMapping::distLongerThanBorder)
						{
							// Wall is directly in front of us
							frontWallsDetected++;
						}
					}
				}
			}
			else
			{
				if (tempFusedData.distSensorState.frontLeft == DistSensorStatus::underflow)
				{
					frontWallsDetected++;
				}
			}

			if (tempFusedData.distSensorState.frontRight == DistSensorStatus::ok)
			{
				// Measurement is ok
				// Check if resulting hit point is a 90° wall in front of us
				if (tempFusedData.robotState.heading == AbsoluteDir::north || tempFusedData.robotState.heading == AbsoluteDir::south)
				{
					float hitY = headingSin * tempFusedData.distances.frontRight / 10.0f + tempFusedData.robotState.position.y + sinf(tempFusedData.robotState.globalHeading - JAFDSettings::Mechanics::distSensFrontAngleToMiddle) * JAFDSettings::Mechanics::distSensFrontDistToMiddle;

					if (fabsf(hitY - tempFusedData.robotState.mapCoordinate.y * JAFDSettings::Field::cellWidth) < JAFDSettings::MazeMapping::widthSecureDetectFactor * JAFDSettings::Field::cellWidth / 2.0f)
					{
						// Helper variables for trig-calculations
						float cos1 = headingCos * tempFusedData.distances.frontRight / 10.0f;
						float cos2 = cosf(tempFusedData.robotState.globalHeading - JAFDSettings::Mechanics::distSensFrontAngleToMiddle) * JAFDSettings::Mechanics::distSensFrontDistToMiddle;

						float hitX = cos1 + tempFusedData.robotState.position.x + cos2;

						if (fabsf(hitX - tempFusedData.robotState.mapCoordinate.x * JAFDSettings::Field::cellWidth) < JAFDSettings::Field::cellWidth / 2.0f + JAFDSettings::MazeMapping::distLongerThanBorder)
						{
							// Wall is directly in front of us
							frontWallsDetected++;
						}
					}
				}
				else
				{
					float hitX = headingCos * tempFusedData.distances.frontRight / 10.0f + tempFusedData.robotState.position.x + cosf(tempFusedData.robotState.globalHeading - JAFDSettings::Mechanics::distSensFrontAngleToMiddle) * JAFDSettings::Mechanics::distSensFrontDistToMiddle;

					if (fabsf(hitX - tempFusedData.robotState.mapCoordinate.x * JAFDSettings::Field::cellWidth) < JAFDSettings::MazeMapping::widthSecureDetectFactor * JAFDSettings::Field::cellWidth / 2.0f)
					{
						// Helper variables for trig-calculation
						float sin1 = headingSin * tempFusedData.distances.frontRight / 10.0f;
						float sin2 = sinf(tempFusedData.robotState.globalHeading - JAFDSettings::Mechanics::distSensFrontAngleToMiddle) * JAFDSettings::Mechanics::distSensFrontDistToMiddle;

						float hitY = sin1 + tempFusedData.robotState.position.y + sin2;

						if (fabsf(hitY - tempFusedData.robotState.mapCoordinate.y * JAFDSettings::Field::cellWidth) < JAFDSettings::Field::cellWidth / 2.0f + JAFDSettings::MazeMapping::distLongerThanBorder)
						{
							// Wall is directly in front of us
							frontWallsDetected++;
						}
					}
				}
			}
			else
			{
				if (tempFusedData.distSensorState.frontRight == DistSensorStatus::underflow)
				{
					frontWallsDetected++;
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
						float hitX = -headingSin * tempFusedData.distances.leftFront / 10.0f + tempFusedData.robotState.position.x - sinf(tempFusedData.robotState.globalHeading - JAFDSettings::Mechanics::distSensLRAngleToMiddle) * JAFDSettings::Mechanics::distSensLRDistToMiddle;

						if (fabsf(hitX - tempFusedData.robotState.mapCoordinate.x * JAFDSettings::Field::cellWidth) < JAFDSettings::MazeMapping::widthSecureDetectFactor * JAFDSettings::Field::cellWidth / 2.0f)
						{
							// Wall is directly left of us
							leftWallsDetected++;
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
						float hitY = headingCos * tempFusedData.distances.leftFront / 10.0f + tempFusedData.robotState.position.y + cosf(tempFusedData.robotState.globalHeading + JAFDSettings::Mechanics::distSensLRAngleToMiddle) * JAFDSettings::Mechanics::distSensLRDistToMiddle;

						if (fabsf(hitY - tempFusedData.robotState.mapCoordinate.y * JAFDSettings::Field::cellWidth) < JAFDSettings::MazeMapping::widthSecureDetectFactor * JAFDSettings::Field::cellWidth / 2.0f)
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
				// Check if resulting hit point is a 90° wall left of us
				if (tempFusedData.robotState.heading == AbsoluteDir::north || tempFusedData.robotState.heading == AbsoluteDir::south)
				{
					float cos1 = headingCos * tempFusedData.distances.leftBack / 10.0f;
					float cos2 = cosf(tempFusedData.robotState.globalHeading + JAFDSettings::Mechanics::distSensLRAngleToMiddle) * JAFDSettings::Mechanics::distSensLRDistToMiddle;

					float hitY = cos1 + tempFusedData.robotState.position.y + cos2;

					if (fabsf(hitY - tempFusedData.robotState.mapCoordinate.y * JAFDSettings::Field::cellWidth) < JAFDSettings::Field::cellWidth / 2.0f + JAFDSettings::MazeMapping::distLongerThanBorder)
					{
						float hitX = -headingSin * tempFusedData.distances.leftBack / 10.0f + tempFusedData.robotState.position.x - sinf(tempFusedData.robotState.globalHeading + JAFDSettings::Mechanics::distSensLRAngleToMiddle) * JAFDSettings::Mechanics::distSensLRDistToMiddle;

						if (fabsf(hitX - tempFusedData.robotState.mapCoordinate.x * JAFDSettings::Field::cellWidth) < JAFDSettings::MazeMapping::widthSecureDetectFactor * JAFDSettings::Field::cellWidth / 2.0f)
						{
							// Wall is directly left of us
							leftWallsDetected++;
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
						float hitY = headingCos * tempFusedData.distances.leftBack / 10.0f + tempFusedData.robotState.position.y + cosf(tempFusedData.robotState.globalHeading + JAFDSettings::Mechanics::distSensLRAngleToMiddle) * JAFDSettings::Mechanics::distSensLRDistToMiddle;

						if (fabsf(hitY - tempFusedData.robotState.mapCoordinate.y * JAFDSettings::Field::cellWidth) < JAFDSettings::MazeMapping::widthSecureDetectFactor * JAFDSettings::Field::cellWidth / 2.0f)
						{
							// Wall is directly left of us
							leftWallsDetected++;
						}
					}
				}
			}
			else if (tempFusedData.distSensorState.leftBack == DistSensorStatus::underflow)
			{
				leftWallsDetected++;
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
						float hitX = headingSin * tempFusedData.distances.rightFront / 10.0f + tempFusedData.robotState.position.x + sinf(tempFusedData.robotState.globalHeading + JAFDSettings::Mechanics::distSensLRAngleToMiddle) * JAFDSettings::Mechanics::distSensLRDistToMiddle;

						if (fabsf(hitX - tempFusedData.robotState.mapCoordinate.x * JAFDSettings::Field::cellWidth) < JAFDSettings::MazeMapping::widthSecureDetectFactor * JAFDSettings::Field::cellWidth / 2.0f)
						{
							// Wall is directly right of us
							rightWallsDetected++;
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
						float hitY = -headingCos * tempFusedData.distances.rightFront / 10.0f + tempFusedData.robotState.position.y - cosf(tempFusedData.robotState.globalHeading + JAFDSettings::Mechanics::distSensLRAngleToMiddle) * JAFDSettings::Mechanics::distSensLRDistToMiddle;

						if (fabsf(hitY - tempFusedData.robotState.mapCoordinate.y * JAFDSettings::Field::cellWidth) < JAFDSettings::MazeMapping::widthSecureDetectFactor * JAFDSettings::Field::cellWidth / 2.0f)
						{
							// Wall is directly right of us
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
				// Check if resulting hit point is a 90° wall right of us
				if (tempFusedData.robotState.heading == AbsoluteDir::north || tempFusedData.robotState.heading == AbsoluteDir::south)
				{
					float cos1 = -headingCos * tempFusedData.distances.rightBack / 10.0f;
					float cos2 = -cosf(tempFusedData.robotState.globalHeading - JAFDSettings::Mechanics::distSensLRAngleToMiddle) * JAFDSettings::Mechanics::distSensLRDistToMiddle;

					float hitY = cos1 + tempFusedData.robotState.position.y + cos2;

					if (fabsf(hitY - tempFusedData.robotState.mapCoordinate.y * JAFDSettings::Field::cellWidth) < JAFDSettings::Field::cellWidth / 2.0f + JAFDSettings::MazeMapping::distLongerThanBorder)
					{
						float hitX = headingSin * tempFusedData.distances.rightBack / 10.0f + tempFusedData.robotState.position.x + sinf(tempFusedData.robotState.globalHeading - JAFDSettings::Mechanics::distSensLRAngleToMiddle) * JAFDSettings::Mechanics::distSensLRDistToMiddle;

						if (fabsf(hitX - tempFusedData.robotState.mapCoordinate.x * JAFDSettings::Field::cellWidth) < JAFDSettings::MazeMapping::widthSecureDetectFactor * JAFDSettings::Field::cellWidth / 2.0f)
						{
							// Wall is directly right of us
							rightWallsDetected++;
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
						float hitY = -headingCos * tempFusedData.distances.rightBack / 10.0f + tempFusedData.robotState.position.y - cosf(tempFusedData.robotState.globalHeading - JAFDSettings::Mechanics::distSensLRAngleToMiddle) * JAFDSettings::Mechanics::distSensLRDistToMiddle;

						if (fabsf(hitY - tempFusedData.robotState.mapCoordinate.y * JAFDSettings::Field::cellWidth) < JAFDSettings::MazeMapping::widthSecureDetectFactor * JAFDSettings::Field::cellWidth / 2.0f)
						{
							// Wall is directly right of us
							rightWallsDetected++;
						}
					}
				}
			}
			else if (tempFusedData.distSensorState.rightBack == DistSensorStatus::underflow)
			{
				rightWallsDetected++;
			}

			GridCell newCell;
			static int consecutiveOk = 0;

			//Serial.print("before: ");
			//Serial.println(tempFusedData.gridCell.cellConnections, BIN);

			bool isOk = MazeMapping::manageDetectedWalls(frontWallsDetected, leftWallsDetected, rightWallsDetected, tempFusedData, newCell);
			tempFusedData.gridCell = newCell;

			if (isOk) {
				consecutiveOk++;
			}
			else {
				consecutiveOk = 0;
			}

			if (consecutiveOk > 10) {
				MazeMapping::setGridCell(newCell, fusedData.robotState.mapCoordinate);
				//Serial.println("<" + String(frontWallsDetected) + ", " + String(leftWallsDetected) + ", " + String(rightWallsDetected) + ">");
				//Serial.println(newCell.cellConnections, BIN);
			}

			__disable_irq();
			fusedData = tempFusedData;
			__enable_irq();

			return consecutiveOk > 10;
		}

		void calcOffsetAngleFromDistSens() {
			struct {
				bool lf = false;
				bool lb = false;
				bool rf = false;
				bool rb = false;
				bool fl = false;
				bool fr = false;
			} usableData;

			struct {
				int lf = 0;
				int lb = 0;
				int rf = 0;
				int rb = 0;
				int fl = 0;
				int fr = 0;
			} distances;

			auto tempFusedData = fusedData;

			if ((tempFusedData.distSensorState.leftFront == DistSensorStatus::ok &&
				tempFusedData.distances.leftFront < 300 - JAFDSettings::Mechanics::distSensLeftRightDist * 10 * 0.9) ||
				tempFusedData.distSensorState.leftFront == DistSensorStatus::underflow) {
				usableData.lf = true;
				distances.lf = tempFusedData.distSensorState.leftFront == DistSensorStatus::underflow ? DistanceSensors::VL6180::minDist : tempFusedData.distances.leftFront;
			}

			if ((tempFusedData.distSensorState.leftBack == DistSensorStatus::ok &&
				tempFusedData.distances.leftBack < 300 - JAFDSettings::Mechanics::distSensLeftRightDist * 10 * 0.9) ||
				tempFusedData.distSensorState.leftBack == DistSensorStatus::underflow) {
				usableData.lb = true;
				distances.lb = tempFusedData.distSensorState.leftBack == DistSensorStatus::underflow ? DistanceSensors::VL6180::minDist : tempFusedData.distances.leftBack;
			}

			if ((tempFusedData.distSensorState.rightFront == DistSensorStatus::ok &&
				tempFusedData.distances.rightFront < 300 - JAFDSettings::Mechanics::distSensLeftRightDist * 10 * 0.9) ||
				tempFusedData.distSensorState.rightFront == DistSensorStatus::underflow) {
				usableData.rf = true;
				distances.rf = tempFusedData.distSensorState.rightFront == DistSensorStatus::underflow ? DistanceSensors::VL6180::minDist : tempFusedData.distances.rightFront;
			}

			if ((tempFusedData.distSensorState.rightBack == DistSensorStatus::ok &&
				tempFusedData.distances.rightBack < 300 - JAFDSettings::Mechanics::distSensLeftRightDist * 10 * 0.9) ||
				tempFusedData.distSensorState.rightBack == DistSensorStatus::underflow) {
				usableData.rb = true;
				distances.rb = tempFusedData.distSensorState.rightBack == DistSensorStatus::underflow ? DistanceSensors::VL6180::minDist : tempFusedData.distances.rightBack;
			}

			if ((tempFusedData.distSensorState.frontLeft == DistSensorStatus::ok &&
				tempFusedData.distances.frontLeft < 300 - JAFDSettings::Mechanics::distSensFrontBackDist * 10 * 0.9) ||
				tempFusedData.distSensorState.frontLeft == DistSensorStatus::underflow) {
				usableData.fl = true;
				distances.fl = tempFusedData.distSensorState.frontLeft == DistSensorStatus::underflow ? DistanceSensors::VL53L0::minDist : tempFusedData.distances.frontLeft;
			}

			if ((tempFusedData.distSensorState.frontRight == DistSensorStatus::ok &&
				tempFusedData.distances.frontRight < 300 - JAFDSettings::Mechanics::distSensFrontBackDist * 10 * 0.9) ||
				tempFusedData.distSensorState.frontRight == DistSensorStatus::underflow) {
				usableData.fr = true;
				distances.fr = tempFusedData.distSensorState.frontRight == DistSensorStatus::underflow ? DistanceSensors::VL53L0::minDist : tempFusedData.distances.frontRight;
			}

			float angleL = 0.0f;
			float angleR = 0.0f;
			float angleF = 0.0f;

			float distToWallL = -1.0f;
			float distToWallR = -1.0f;
			float distToWallF = -1.0f;

			int numData = 0;

			if (usableData.lf && usableData.lb) {
				calcAngleWallOffsetFromTwoDistances(&angleL, &distToWallL, distances.lf, distances.lb, JAFDSettings::Mechanics::distSensLRSpacing, JAFDSettings::Mechanics::distSensLeftRightDist);
				angleL *= -1.0f;

				numData++;
			}

			if (usableData.rf && usableData.rb) {
				calcAngleWallOffsetFromTwoDistances(&angleR, &distToWallR, distances.rf, distances.rb, JAFDSettings::Mechanics::distSensLRSpacing, JAFDSettings::Mechanics::distSensLeftRightDist);

				numData++;
			}

			if (usableData.fl && usableData.fr) {
				calcAngleWallOffsetFromTwoDistances(&angleF, &distToWallF, distances.fl, distances.fr, JAFDSettings::Mechanics::distSensFrontSpacing, JAFDSettings::Mechanics::distSensFrontBackDist);
				angleF *= -1.0f;

				numData++;
			}

			distToWalls.l = distToWallL;
			distToWalls.r = distToWallR;
			distToWalls.f = distToWallF;

			if (numData > 0) {
				float angle = (angleL + angleR + angleF) / numData;

				float wallAngle = 0.0f;
				switch (tempFusedData.robotState.heading)
				{
				case AbsoluteDir::north:
					wallAngle = 0.0f;
					break;
				case AbsoluteDir::east:
					wallAngle = -M_PI_2;
					break;
				case AbsoluteDir::south:
					wallAngle = M_PI;
					break;
				case AbsoluteDir::west:
					wallAngle = M_PI_2;
					break;
				default:
					break;
				}

				distSensAngle = fitAngleToInterval(angle + wallAngle);
				distSensAngleTrust = (numData + 2.0f) / 5.0f;
			}
			else {
				distSensAngleTrust = 0.0f;
			}
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

			// Border detection (wall with any offset front/back or length)
			uint8_t leftBorderDetected = 0;		// How many times did a border left of us get detected
			uint8_t rightBorderDetected = 0;		// How many times did a border right of us get detected
			uint8_t frontBorderDetected = 0;

			struct
			{
				bool lf = false;
				bool lb = false;
				bool rf = false;
				bool rb = false;
			} borderDetected;

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
							frontBorderDetected++;
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
							frontBorderDetected++;
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
							frontBorderDetected++;
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
							frontBorderDetected++;
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
					}
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
					}
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
					}
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
					}
				}
			}

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

			if (fabsf(tempFusedData.robotState.pitch) > JAFDSettings::SensorFusion::maxPitchForDistSensor)
			{
				lastLeftDist = 0;
				lastRightDist = 0;
			}

			if (validDistSpeedSamples > 0)
			{
				distSensSpeedTrust = validDistSpeedSamples / 3.0f;
				distSensSpeed = (-tempDistSensSpeed / (float)(validDistSpeedSamples)) * JAFDSettings::SensorFusion::distSensSpeedIIRFactor + distSensSpeed * (1.0f - JAFDSettings::SensorFusion::distSensSpeedIIRFactor);	// Negative, because increasing distance means driving away; 
			}
			else
			{
				distSensSpeedTrust = 0.0f;
			}

			calcOffsetAngleFromDistSens();

			lastTime = now;
		}

		void updatePosAndRotFromDist() {
			float totAngleWeight = 0.0f;
			float avgCos = 0.0f;
			float avgSin = 0.0f;
			float avgX = 0.0f;
			int totXNum = 0.0f;
			float avgY = 0.0f;
			int totYNum = 0.0f;

			auto tempRobotState = fusedData.robotState;

			for (int i = 0; i < 10; i++) {
				DistanceSensors::updateDistSensors();
				calcOffsetAngleFromDistSens();

				if (distSensAngleTrust > 0.01f) {
					totAngleWeight += distSensAngleTrust;
					avgCos += cosf(distSensAngle) * distSensAngleTrust;
					avgSin += sinf(distSensAngle) * distSensAngleTrust;
				}

				switch (tempRobotState.heading)
				{
				case AbsoluteDir::north:
					if (distToWalls.l > 0.0f) {
						avgY += (tempRobotState.mapCoordinate.y + 0.5f) * JAFDSettings::Field::cellWidth - distToWalls.l;
						totYNum++;
					}

					if (distToWalls.r > 0.0f) {
						avgY += (tempRobotState.mapCoordinate.y - 0.5f) * JAFDSettings::Field::cellWidth + distToWalls.r;
						totYNum++;
					}

					if (distToWalls.f > 0.0f) {
						avgX += (tempRobotState.mapCoordinate.x + 0.5f) * JAFDSettings::Field::cellWidth - distToWalls.f;
						totXNum++;
					}
					break;
				case AbsoluteDir::east:
					if (distToWalls.l > 0.0f) {
						avgX += (tempRobotState.mapCoordinate.x + 0.5f) * JAFDSettings::Field::cellWidth - distToWalls.l;
						totXNum++;
					}

					if (distToWalls.r > 0.0f) {
						avgX += (tempRobotState.mapCoordinate.x - 0.5f) * JAFDSettings::Field::cellWidth + distToWalls.r;
						totXNum++;
					}

					if (distToWalls.f > 0.0f) {
						avgY += (tempRobotState.mapCoordinate.y - 0.5f) * JAFDSettings::Field::cellWidth + distToWalls.f;
						totYNum++;
					}
					break;
				case AbsoluteDir::south:
					if (distToWalls.l > 0.0f) {
						avgY += (tempRobotState.mapCoordinate.y - 0.5f) * JAFDSettings::Field::cellWidth + distToWalls.l;
						totYNum++;
					}

					if (distToWalls.r > 0.0f) {
						avgY += (tempRobotState.mapCoordinate.y + 0.5f) * JAFDSettings::Field::cellWidth - distToWalls.r;
						totYNum++;
					}

					if (distToWalls.f > 0.0f) {
						avgX += (tempRobotState.mapCoordinate.x - 0.5f) * JAFDSettings::Field::cellWidth + distToWalls.f;
						totXNum++;
					}
					break;
				case AbsoluteDir::west:
					if (distToWalls.l > 0.0f) {
						avgX += (tempRobotState.mapCoordinate.x - 0.5f) * JAFDSettings::Field::cellWidth + distToWalls.l;
						totXNum++;
					}

					if (distToWalls.r > 0.0f) {
						avgX += (tempRobotState.mapCoordinate.x + 0.5f) * JAFDSettings::Field::cellWidth - distToWalls.r;
						totXNum++;
					}

					if (distToWalls.f > 0.0f) {
						avgY += (tempRobotState.mapCoordinate.y + 0.5f) * JAFDSettings::Field::cellWidth - distToWalls.f;
						totYNum++;
					}
					break;
				default:
					break;
				}
			}

			//Serial.println("......");
			//Serial.println(distToWalls.f);
			//Serial.println(distToWalls.l);
			//Serial.println(distToWalls.r);
			//Serial.println("......");

			noInterrupts();

			correctedState.zeroPitch = true;

			if (totXNum > 0) {
				avgX /= totXNum;
				correctedState.x = avgX;
				correctedState.newX = true;
			}

			if (totYNum > 0) {
				avgY /= totYNum;
				correctedState.y = avgY;
				correctedState.newY = true;
			}

			float avgAngle = 0.0f;
			if (totAngleWeight > 0.01f) {
				avgCos /= totAngleWeight;
				avgSin /= totAngleWeight;
				avgAngle = atan2f(avgSin, avgCos);

				correctedState.heading = makeRotationCoherent(tempRobotState.globalHeading, avgAngle);
				correctedState.newHeading = true;

				float currentRotEncAngle = (MotorControl::getDistance(Motor::right) - MotorControl::getDistance(Motor::left)) / (JAFDSettings::Mechanics::wheelDistToMiddle * 2.0f * JAFDSettings::MotorControl::magicFactor);
				totalHeadingOff = fitAngleToInterval(avgAngle - currentRotEncAngle);
			}

			interrupts();

			if (totAngleWeight > 0.01f) {
				Bno055::tare(avgAngle);
			}
		}

		FusedData getFusedData()
		{
			return fusedData;
		}

		void updateSensors()
		{
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

		float getAngleRelToWall() {
			return distSensAngleTrust > 0.01f ? distSensAngle : NAN;
		}
	}
}