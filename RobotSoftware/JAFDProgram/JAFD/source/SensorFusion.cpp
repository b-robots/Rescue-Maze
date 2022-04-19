/*
This file of the library is responsible for the sensor fusion
*/

#pragma once

#include "../header/MazeMapping.h"
#include "../header/Math.h"
#include "../header/SensorFusion.h"
#include "../header/SmoothDriving.h"
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

		volatile bool forceAnglePosReset = false;
		volatile bool bnoErr = false;

		void sensorFiltering(const uint8_t freq)
		{
			float correctedDistSensAngleTrust = SmoothDriving::isDrivingStraight() ? distSensAngleTrust : 0.0f;
			float correctedDistSensSpeedTrust = SmoothDriving::isDrivingStraight() ? distSensSpeedTrust : 0.0f;

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

			if (!bnoErr)
			{
				tempRobotState.pitch = getPitch(bnoForwardVec) * JAFDSettings::SensorFusion::pitchIIRFactor + lastPitch * (1.0f - JAFDSettings::SensorFusion::pitchIIRFactor);
			}

			tempRobotState.angularVel.z = (tempRobotState.pitch - lastPitch) * freq;

			float trustYawVel = 0.0f;

			if (trustWheels)
			{
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

				currHeading = interpolateAngle(currHeading, fitAngleToInterval(distSensAngle), correctedDistSensAngleTrust * JAFDSettings::SensorFusion::distAngularPortion);
			}
			else
			{
				if (!bnoErr) {
					trustYawVel = 0.6f;
					tempRobotState.angularVel.x = fitAngleToInterval(bnoHeading - lastBnoHeading) * freq;
					currHeading = interpolateAngle(bnoHeading, fitAngleToInterval(distSensAngle), sqrtf(correctedDistSensAngleTrust * JAFDSettings::SensorFusion::distAngularPortion * (1.0f - JAFDSettings::SensorFusion::bno055RotPortion)));
				}
				else if (correctedDistSensAngleTrust > 0.001f) {
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

			tempRobotState.globalHeading = makeRotationCoherent(lastHeading, fitAngleToInterval(currHeading));

			tempRobotState.forwardVec = toForwardVec(tempRobotState.globalHeading, tempRobotState.pitch);		// Calculate forward vector

			// Linear velocitys
			tempRobotState.forwardVel = ((tempRobotState.wheelSpeeds.left + tempRobotState.wheelSpeeds.right) / 2.0f) * (1.0f - correctedDistSensSpeedTrust * JAFDSettings::SensorFusion::distSpeedPortion) + distSensSpeed * (correctedDistSensSpeedTrust * JAFDSettings::SensorFusion::distSpeedPortion);
			tempRobotState.forwardVel *= JAFDSettings::MotorControl::magicFactor * 0.98f;
			tempRobotState.forwardVel = distSensSpeed * (correctedDistSensSpeedTrust * JAFDSettings::SensorFusion::distSpeedPortion) + tempRobotState.forwardVel * (1.0f - correctedDistSensSpeedTrust * JAFDSettings::SensorFusion::distSpeedPortion);

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

			// Map absolute heading & position
			currHeading = fitAngleToInterval(tempRobotState.globalHeading);
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

		bool scanSurrounding(uint8_t& outCumSureWalls) {
			uint8_t frontWallsDetected = 0;		// How many times did a wall in front of us get detected
			uint8_t leftWallsDetected = 0;		// How many times did a wall left of us get detected
			uint8_t rightWallsDetected = 0;		// How many times did a wall right of us get detected

			static MapCoordinate lastCoordinate;

			auto tempFusedData = fusedData;

			// positive value -> too far in the given direction
			float centerOffsetToLeft = 0.0f;
			float centerOffsetToFront = 0.0f;

			switch (tempFusedData.robotState.heading)
			{
			case AbsoluteDir::north:
				centerOffsetToFront = tempFusedData.robotState.position.x - roundf(tempFusedData.robotState.position.x / 30.0f) * 30.0f;
				centerOffsetToLeft = tempFusedData.robotState.position.y - roundf(tempFusedData.robotState.position.y / 30.0f) * 30.0f;
				break;
			case AbsoluteDir::east:
				centerOffsetToFront = roundf(tempFusedData.robotState.position.y / 30.0f) * 30.0f - tempFusedData.robotState.position.y;
				centerOffsetToLeft = tempFusedData.robotState.position.x - roundf(tempFusedData.robotState.position.x / 30.0f) * 30.0f;
				break;
			case AbsoluteDir::south:
				centerOffsetToFront = roundf(tempFusedData.robotState.position.x / 30.0f) * 30.0f - tempFusedData.robotState.position.x;
				centerOffsetToLeft = roundf(tempFusedData.robotState.position.y / 30.0f) * 30.0f - tempFusedData.robotState.position.y;
				break;
			case AbsoluteDir::west:
				centerOffsetToFront = tempFusedData.robotState.position.y - roundf(tempFusedData.robotState.position.y / 30.0f) * 30.0f;
				centerOffsetToLeft = roundf(tempFusedData.robotState.position.x / 30.0f) * 30.0f - tempFusedData.robotState.position.x;
				break;
			default:
				break;
			}

			if (tempFusedData.distSensorState.frontLeft == DistSensorStatus::ok)
			{
				if ((tempFusedData.distances.frontLeft / 10.0f + centerOffsetToFront) < JAFDSettings::MazeMapping::distLongerThanBorder + JAFDSettings::Field::cellWidth / 2.0f) {
					frontWallsDetected++;
				}
			}
			else if (tempFusedData.distSensorState.frontLeft == DistSensorStatus::underflow)
			{
				frontWallsDetected++;
			}

			if (tempFusedData.distSensorState.frontRight == DistSensorStatus::ok)
			{
				if ((tempFusedData.distances.frontRight / 10.0f + centerOffsetToFront) < JAFDSettings::MazeMapping::distLongerThanBorder + JAFDSettings::Field::cellWidth / 2.0f) {
					frontWallsDetected++;
				}
			}
			else if (tempFusedData.distSensorState.frontRight == DistSensorStatus::underflow)
			{
				frontWallsDetected++;
			}

			if (tempFusedData.distSensorState.leftFront == DistSensorStatus::ok)
			{
				if ((tempFusedData.distances.leftFront / 10.0f + centerOffsetToLeft) < JAFDSettings::MazeMapping::distLongerThanBorder + JAFDSettings::Field::cellWidth / 2.0f) {
					leftWallsDetected++;
				}
			}
			else if (tempFusedData.distSensorState.leftFront == DistSensorStatus::underflow)
			{
				leftWallsDetected++;
			}

			if (tempFusedData.distSensorState.leftBack == DistSensorStatus::ok)
			{
				if ((tempFusedData.distances.leftBack / 10.0f + centerOffsetToLeft) < JAFDSettings::MazeMapping::distLongerThanBorder + JAFDSettings::Field::cellWidth / 2.0f) {
					leftWallsDetected++;
				}
			}
			else if (tempFusedData.distSensorState.leftBack == DistSensorStatus::underflow)
			{
				leftWallsDetected++;
			}

			if (tempFusedData.distSensorState.rightFront == DistSensorStatus::ok)
			{
				if ((tempFusedData.distances.rightFront / 10.0f - centerOffsetToLeft) < JAFDSettings::MazeMapping::distLongerThanBorder + JAFDSettings::Field::cellWidth / 2.0f) {
					rightWallsDetected++;
				}
			}
			else if (tempFusedData.distSensorState.rightFront == DistSensorStatus::underflow)
			{
				rightWallsDetected++;
			}

			if (tempFusedData.distSensorState.rightBack == DistSensorStatus::ok)
			{
				if ((tempFusedData.distances.rightBack / 10.0f - centerOffsetToLeft) < JAFDSettings::MazeMapping::distLongerThanBorder + JAFDSettings::Field::cellWidth / 2.0f) {
					rightWallsDetected++;
				}
			}
			else if (tempFusedData.distSensorState.rightBack == DistSensorStatus::underflow)
			{
				rightWallsDetected++;
			}

			GridCell newCell;
			uint8_t sureWalls;
			static int consecutiveOk = 0;

			bool isOk = MazeMapping::manageDetectedWalls(frontWallsDetected, leftWallsDetected, rightWallsDetected, tempFusedData, newCell, sureWalls);
			tempFusedData.gridCell = newCell;

			if (isOk) {
				consecutiveOk++;
			}
			else {
				consecutiveOk = 0;
			}

			if (consecutiveOk >= 5) {
				Serial.print("successful scan: ");
				Serial.print(sureWalls, BIN);
				Serial.print(", at: ");
				Serial.print(tempFusedData.robotState.mapCoordinate.x);
				Serial.print(", ");
				Serial.println(tempFusedData.robotState.mapCoordinate.y);

				outCumSureWalls |= sureWalls;
			}

			__disable_irq();
			fusedData = tempFusedData;
			__enable_irq();

			lastCoordinate = tempFusedData.robotState.mapCoordinate;

			return consecutiveOk >= 5;
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

			// overflow / underflow leads to min and max
			struct {
				float lf = 0.0f;
				float lb = 0.0f;
				float rf = 0.0f;
				float rb = 0.0f;
				float fl = 0.0f;
				float fr = 0.0f;
			} distances;

			float headingCos = cosf(tempFusedData.robotState.globalHeading);
			float headingSin = sinf(tempFusedData.robotState.globalHeading);

			if (tempFusedData.distSensorState.frontLeft == DistSensorStatus::ok)
			{
				distances.fl = tempFusedData.distances.frontLeft;
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
				if (tempFusedData.distSensorState.frontLeft == DistSensorStatus::underflow) {
					distances.fl = DistanceSensors::frontLeft.minDist;
				}
				else if (tempFusedData.distSensorState.frontLeft == DistSensorStatus::overflow) {
					distances.fl = DistanceSensors::frontLeft.maxDist;
				}

				lastLeftDist = 0;
			}

			if (tempFusedData.distSensorState.frontRight == DistSensorStatus::ok)
			{
				distances.fr = tempFusedData.distances.frontRight;

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
				if (tempFusedData.distSensorState.frontLeft == DistSensorStatus::underflow) {
					distances.fr = DistanceSensors::frontRight.minDist;
				}
				else if (tempFusedData.distSensorState.frontLeft == DistSensorStatus::overflow) {
					distances.fr = DistanceSensors::frontRight.maxDist;
				}

				lastRightDist = 0;
			}

			if (tempFusedData.distSensorState.leftFront == DistSensorStatus::ok)
			{
				distances.lf = tempFusedData.distances.leftFront;
			}
			else if (tempFusedData.distSensorState.leftFront == DistSensorStatus::underflow) {
				distances.lf = DistanceSensors::leftFront.minDist;
			}
			else if (tempFusedData.distSensorState.leftFront == DistSensorStatus::overflow) {
				distances.lf = DistanceSensors::leftFront.maxDist;
			}

			if (tempFusedData.distSensorState.leftBack == DistSensorStatus::ok)
			{
				distances.lb = tempFusedData.distances.leftBack;
			}
			else if (tempFusedData.distSensorState.leftBack == DistSensorStatus::underflow) {
				distances.lb = DistanceSensors::leftBack.minDist;
			}
			else if (tempFusedData.distSensorState.leftBack == DistSensorStatus::overflow) {
				distances.lb = DistanceSensors::leftBack.maxDist;
			}

			if (tempFusedData.distSensorState.rightFront == DistSensorStatus::ok)
			{
				distances.rf = tempFusedData.distances.rightFront;
			}
			else if (tempFusedData.distSensorState.rightFront == DistSensorStatus::underflow) {
				distances.rf = DistanceSensors::rightFront.minDist;
			}
			else if (tempFusedData.distSensorState.rightFront == DistSensorStatus::overflow) {
				distances.rf = DistanceSensors::rightFront.maxDist;
			}

			if (tempFusedData.distSensorState.rightBack == DistSensorStatus::ok)
			{
				distances.rb = tempFusedData.distances.rightBack;
			}
			else if (tempFusedData.distSensorState.rightBack == DistSensorStatus::underflow) {
				distances.rb = DistanceSensors::rightBack.minDist;
			}
			else if (tempFusedData.distSensorState.rightBack == DistSensorStatus::overflow) {
				distances.rb = DistanceSensors::rightBack.maxDist;
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

			static bool firstEdgeDetection = true;
			static decltype(distances) lastDistances;

			if (!(SmoothDriving::isDrivingStraight() &&
				MotorControl::getSpeeds().left > 0 &&
				MotorControl::getSpeeds().right > 0 &&
				tempFusedData.robotState.pitch < JAFDSettings::SensorFusion::maxPitchForDistSensor)) {
				firstEdgeDetection = true;
			}
			else {
				if (firstEdgeDetection) {
					firstEdgeDetection = false;
				}
				else {
					float x = NAN;
					float y = NAN;

					if ((fabsf(lastDistances.lf - distances.lf) > 50 && lastDistances.lf * distances.lf > 0.1f) || (fabsf(lastDistances.rf - distances.rf) > 50 && lastDistances.rf * distances.rf > 0.1f)) {
						Serial.println("front: ");
						Serial.println(lastDistances.lf);
						Serial.println(distances.lf);
						Serial.println(lastDistances.rf);
						Serial.println(distances.rf);

						switch (tempFusedData.robotState.heading)
						{
						case AbsoluteDir::north:
						{
							float edgeX = roundf((tempFusedData.robotState.position.x + JAFDSettings::Mechanics::distSensLRSpacing / 2.0f - 15.0f) / 30.0f) * 30.0f + 15.0f;
							x = edgeX - JAFDSettings::Mechanics::distSensLRSpacing / 2.0f;
							break;
						}
						case AbsoluteDir::east:
						{
							float edgeY = roundf((tempFusedData.robotState.position.y - JAFDSettings::Mechanics::distSensLRSpacing / 2.0f - 15.0f) / 30.0f) * 30.0f + 15.0f;
							y = edgeY + JAFDSettings::Mechanics::distSensLRSpacing / 2.0f;
							break;
						}
						case AbsoluteDir::south:
						{
							float edgeX = roundf((tempFusedData.robotState.position.x - JAFDSettings::Mechanics::distSensLRSpacing / 2.0f - 15.0f) / 30.0f) * 30.0f + 15.0f;
							x = edgeX + JAFDSettings::Mechanics::distSensLRSpacing / 2.0f;
							break;
						}
						case AbsoluteDir::west:
						{
							float edgeY = roundf((tempFusedData.robotState.position.y + JAFDSettings::Mechanics::distSensLRSpacing / 2.0f - 15.0f) / 30.0f) * 30.0f + 15.0f;
							y = edgeY - JAFDSettings::Mechanics::distSensLRSpacing / 2.0f;
							break;
						}
						default:
							break;
						}
					}

					if ((fabsf(lastDistances.lb - distances.lb) > 50 && lastDistances.lb * distances.lb > 0.1f) || (fabsf(lastDistances.rb - distances.rb) > 50 && lastDistances.rb * distances.rb > 0.1f)) {
						Serial.println("back: ");
						Serial.println(lastDistances.lb);
						Serial.println(distances.lb);
						Serial.println(lastDistances.rb);
						Serial.println(distances.rb);

						switch (tempFusedData.robotState.heading)
						{
						case AbsoluteDir::north:
						{
							float edgeX = roundf((tempFusedData.robotState.position.x - JAFDSettings::Mechanics::distSensLRSpacing / 2.0f - 15.0f) / 30.0f) * 30.0f + 15.0f;
							x = edgeX + JAFDSettings::Mechanics::distSensLRSpacing / 2.0f;
							break;
						}
						case AbsoluteDir::east:
						{
							float edgeY = roundf((tempFusedData.robotState.position.y + JAFDSettings::Mechanics::distSensLRSpacing / 2.0f - 15.0f) / 30.0f) * 30.0f + 15.0f;
							y = edgeY - JAFDSettings::Mechanics::distSensLRSpacing / 2.0f;
							break;
						}
						case AbsoluteDir::south:
						{
							float edgeX = roundf((tempFusedData.robotState.position.x + JAFDSettings::Mechanics::distSensLRSpacing / 2.0f - 15.0f) / 30.0f) * 30.0f + 15.0f;
							x = edgeX - JAFDSettings::Mechanics::distSensLRSpacing / 2.0f;
							break;
						}
						case AbsoluteDir::west:
						{
							float edgeY = roundf((tempFusedData.robotState.position.y - JAFDSettings::Mechanics::distSensLRSpacing / 2.0f - 15.0f) / 30.0f) * 30.0f + 15.0f;
							y = edgeY + JAFDSettings::Mechanics::distSensLRSpacing / 2.0f;
							break;
						}
						default:
							break;
						}
					}

					if (!std::isnan(x)) {
						Serial.print("new edge x: ");
						Serial.print(x);
						Serial.print(", pos.x: ");
						Serial.println(tempFusedData.robotState.position.x);
						__disable_irq();
						correctedState.x = x * 0.8f + tempFusedData.robotState.position.x * 0.2f;
						correctedState.newX = true;
						__enable_irq();
					}
					else if (!std::isnan(y)) {
						Serial.print("new edge y: ");
						Serial.print(y);
						Serial.print(", pos.y: ");
						Serial.println(tempFusedData.robotState.position.y);
						__disable_irq();
						correctedState.y = y * 0.8f + tempFusedData.robotState.position.y * 0.2f;
						correctedState.newY = true;
						__enable_irq();
					}
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

			__disable_irq();
			if (forceAnglePosReset) {
				forceAnglePosReset = false;
				__enable_irq();
				updatePosAndRotFromDist();
			}
			else {
				__enable_irq();
			}

			lastDistances = distances;
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

			__disable_irq();

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
				totalHeadingOff = fitAngleToInterval(currentRotEncAngle - avgAngle);
			}

			__enable_irq();

			if (totAngleWeight > 0.01f) {
				Bno055::tare(avgAngle);
			}
		}

		FusedData getFusedData()
		{
			__disable_irq();
			FusedData copy = fusedData;
			__enable_irq();
			return copy;
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
			float wallAngle = 0.0f;
			switch (fusedData.robotState.heading)
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

			float angle = distSensAngleTrust > 0.01f ? distSensAngle : NAN;
			return fitAngleToInterval(angle - wallAngle);
		}
	}
}