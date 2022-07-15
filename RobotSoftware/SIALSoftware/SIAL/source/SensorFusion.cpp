#include "../header/MazeMapping.h"
#include "../header/Math.h"
#include "../header/SensorFusion.h"
#include "../header/SmoothDriving.h"
#include "../header/MotorControl.h"
#include "../header/DistanceSensors.h"
#include "../header/Gyro.h"
#include "../header/ColorSensor.h"
#include "../header/SmallThings.h"
#include "../SIALSettings.h"

#include <cmath>
#include <limits>

namespace SIAL
{
	namespace SensorFusion
	{
		namespace
		{
			FusedData fusedData;				// Fused data
			float totalHeadingOff = 0.0f;		// Total heading offsets

			NewForcedFusionValues correctedState;
		}

		float distToLWall = 0.0f;
		float distToRWall = 0.0f;

		bool forceAnglePosReset = false;
		uint16_t consecutiveRotStuck = 0;

		void setCorrectedState(NewForcedFusionValues newValues) {
			correctedState = newValues;
		}

		bool waitForAllDistSens() {
			static int64_t t = -1;
			if (t == -1) {
				t = millis();
			}

			if (t != -1 && (millis() - t > 100)) {
				t = -1;
				return true;
			}

			static DistSensBool cumUpdates = DistSensBool(false);
			cumUpdates = cumUpdates | fusedData.distSensUpdates;
			if (cumUpdates == DistSensBool(true)) {
				cumUpdates = DistSensBool(false);
				t = -1;
				return true;
			}

			return false;
		}

		void sensorFusion(bool firstAgain)
		{
			static bool first = true;
			static uint32_t lastTime = millis();

			if (firstAgain) {
				first = true;
			}

			if (first) {
				first = false;
				lastTime = millis();
				return;
			}

			const float dt = (millis() - lastTime) / 1000.0f;
			lastTime = millis();

			RobotState robotState = fusedData.robotState;

			float correctedDistSensAngleTrust = SmoothDriving::getInformation().drivingStraight ? fusedData.fusedDistSens.distSensAngleTrust : 0.0f;

			robotState.wheelSpeeds = MotorControl::getFloatSpeeds();

			// Rotation
			// We don't handle rotation of robot on ramp (pitch != 0°) completely correct! But it shouldn't matter.
			static float lastBnoHeading = 0.0f;
			const auto lastHeading = robotState.globalHeading;
			const auto lastAngularVel = robotState.angularVel;
			static float lastBnoPitch = 0.0f;
			float currHeading = 0.0f;

			const auto bnoForwardVec = Gyro::getForwardVec();
			auto bnoHeading = getGlobalHeading(bnoForwardVec);

			if (fabsf(fitAngleToInterval(bnoHeading - lastBnoHeading)) / dt > 10.0f) {
				bnoHeading = fitAngleToInterval(lastBnoHeading + robotState.angularVel.x * dt * 0.8f);
			}

			float bnoPitch = getPitch(bnoForwardVec);
			float lastPitch = robotState.pitch;

			robotState.pitch = bnoPitch * SIALSettings::SensorFusion::pitchIIRFactor + robotState.pitch * (1.0f - SIALSettings::SensorFusion::pitchIIRFactor);
			lastBnoPitch = bnoPitch;

			robotState.angularVel.z = (robotState.pitch - lastPitch) / dt;

			float encoderYawVel = (robotState.wheelSpeeds.right - robotState.wheelSpeeds.left) / (SIALSettings::Mechanics::wheelDistToMiddle * 2.0f * SIALSettings::SensorFusion::chi);

			if (SmoothDriving::getInformation().uid == DrivingTaskUID::rotate &&
				fabsf(encoderYawVel - fitAngleToInterval(bnoHeading - lastBnoHeading) / dt) > 0.5f) {
				consecutiveRotStuck++;
			}
			else {
				consecutiveRotStuck = 0;
			}

			robotState.angularVel.x = fitAngleToInterval(bnoHeading - lastBnoHeading) / dt * SIALSettings::SensorFusion::bno055DiffPortion + encoderYawVel * (1.0f - SIALSettings::SensorFusion::bno055DiffPortion);
			robotState.angularVel.y = 0.0f;
			robotState.angularVel = robotState.angularVel * SIALSettings::SensorFusion::angularVelIIRFactor + lastAngularVel * (1.0f - SIALSettings::SensorFusion::angularVelIIRFactor);

			currHeading = (MotorControl::getDistance(Motor::right) - MotorControl::getDistance(Motor::left)) / (SIALSettings::Mechanics::wheelDistToMiddle * 2.0f * SIALSettings::SensorFusion::chi);
			currHeading -= totalHeadingOff;

			currHeading = interpolateAngle(fitAngleToInterval(currHeading), bnoHeading, SIALSettings::SensorFusion::bno055RotPortion);

			currHeading = interpolateAngle(currHeading, fitAngleToInterval(fusedData.fusedDistSens.distSensAngle), correctedDistSensAngleTrust * SIALSettings::SensorFusion::distAngularPortion);

			// mix asolute and relative heading
			currHeading = interpolateAngle(currHeading, fitAngleToInterval(lastHeading + robotState.angularVel.x * dt), SIALSettings::SensorFusion::angleDiffPortion);

			robotState.globalHeading = makeRotationCoherent(lastHeading, fitAngleToInterval(currHeading));

			if (correctedState.newHeading) {
				robotState.globalHeading = correctedState.heading;
				currHeading = correctedState.heading;
				bnoHeading = correctedState.heading;
				lastBnoHeading = correctedState.heading;
				robotState.angularVel.x = 0.0f;
				Serial.println("Reset heading");
				Serial.println(correctedState.heading);
				correctedState.newHeading = false;
			}

			if (correctedState.zeroPitch) {
				robotState.pitch = 0.0f;
				robotState.angularVel.z = 0.0f;
				bnoPitch = 0.0f;
				lastBnoPitch = 0.0f;
				lastPitch = 0.0f;
				correctedState.zeroPitch = false;
			}

			robotState.forwardVec = toForwardVec(robotState.globalHeading, robotState.pitch);		// Calculate forward vector

			// Linear velocitys
			robotState.forwardVel = (robotState.wheelSpeeds.left + robotState.wheelSpeeds.right) / 2.0f * SIALSettings::SensorFusion::speedIIRFac + robotState.forwardVel * (1.0f - SIALSettings::SensorFusion::speedIIRFac);

			// Position
			robotState.position += (Vec2f(robotState.forwardVec)).normalized() * (robotState.forwardVel * dt);

			if (correctedState.newX) {
				robotState.position.x = correctedState.x;
				Serial.print("correct x: ");
				Serial.println(robotState.position.x);
				correctedState.newX = false;
			}

			if (correctedState.newY) {
				robotState.position.y = correctedState.y;
				Serial.print("correct y: ");
				Serial.println(robotState.position.y);
				correctedState.newY = false;
			}

			// NAN detection
			static Vec3f nonNanAngularVel;
			if (!(std::isfinite(robotState.angularVel.x) &&
				std::isfinite(robotState.angularVel.y) &&
				std::isfinite(robotState.angularVel.z))) {
				robotState.angularVel = nonNanAngularVel;
			}
			else {
				nonNanAngularVel = robotState.angularVel;
			}

			static Vec2f nonNanPostion;
			if (!(std::isfinite(robotState.position.x) &&
				std::isfinite(robotState.position.y))) {
				robotState.position = nonNanPostion;
			}
			else {
				nonNanPostion = robotState.position;
			}

			static float nonNanHeading = 0.0f;
			if (!std::isfinite(robotState.globalHeading)) {
				robotState.globalHeading = nonNanHeading;
				bnoHeading = nonNanHeading;
			}
			else {
				nonNanHeading = robotState.globalHeading;
			}

			static float nonNanPitch = 0.0f;
			if (!std::isfinite(robotState.pitch)) {
				robotState.pitch = nonNanPitch;
			}
			else {
				nonNanPitch = robotState.pitch;
			}

			// Map absolute heading & position
			currHeading = fitAngleToInterval(robotState.globalHeading);
			if (currHeading > M_PI_4 && currHeading < M_3PI_4) {
				robotState.heading = AbsoluteDir::west;
			}
			else if (currHeading < -M_PI_4 && currHeading > -M_3PI_4) {
				robotState.heading = AbsoluteDir::east;
			}
			else if (currHeading < M_PI_4 && currHeading > -M_PI_4) {
				robotState.heading = AbsoluteDir::north;
			}
			else {
				robotState.heading = AbsoluteDir::south;
			}

			auto prevCoord = robotState.mapCoordinate;

			robotState.mapCoordinate.x = (int8_t)roundf(robotState.position.x / 30.0f);
			robotState.mapCoordinate.y = (int8_t)roundf(robotState.position.y / 30.0f);

			switch (robotState.heading) {
			case AbsoluteDir::north:
				distToLWall = 15.0f - SIALSettings::Mechanics::distSensLeftRightDist / 2.0f - (robotState.position.y - robotState.mapCoordinate.y * 30.0f);
				distToRWall = 30.0f - SIALSettings::Mechanics::distSensLeftRightDist - distToLWall;
				break;
			case AbsoluteDir::east:
				distToLWall = 15.0f - SIALSettings::Mechanics::distSensLeftRightDist / 2.0f - (robotState.position.x - robotState.mapCoordinate.x * 30.0f);
				distToRWall = 30.0f - SIALSettings::Mechanics::distSensLeftRightDist - distToLWall;
				break;
			case AbsoluteDir::south:
				distToLWall = 15.0f - SIALSettings::Mechanics::distSensLeftRightDist / 2.0f + (robotState.position.y - robotState.mapCoordinate.y * 30.0f);
				distToRWall = 30.0f - SIALSettings::Mechanics::distSensLeftRightDist - distToLWall;
				break;
			case AbsoluteDir::west:
				distToLWall = 15.0f - SIALSettings::Mechanics::distSensLeftRightDist / 2.0f + (robotState.position.x - robotState.mapCoordinate.x * 30.0f);
				distToRWall = 30.0f - SIALSettings::Mechanics::distSensLeftRightDist - distToLWall;
				break;
			default:
				distToLWall = distToRWall = 15.0f - SIALSettings::Mechanics::distSensLeftRightDist / 2.0f;
				break;
			}

			if (correctedState.clearCell) {
				correctedState.clearCell = false;
				fusedData.gridCell = GridCell(0b1111);
			}

			if (robotState.mapCoordinate != prevCoord) {
				fusedData.gridCell = GridCell(0b1111);
			}

			fusedData.robotState = robotState;

			lastBnoHeading = bnoHeading;
		}

		bool scanSurrounding() {
			uint8_t frontWallsDetected = 0;		// How many times did a wall in front of us get detected
			uint8_t leftWallsDetected = 0;		// How many times did a wall left of us get detected
			uint8_t rightWallsDetected = 0;		// How many times did a wall right of us get detected

			if (fusedData.distSensorState.frontLeft == DistSensorStatus::ok)
			{
				if (fusedData.distances.frontLeft / 10.0f < 20.0f - SIALSettings::Mechanics::distSensFrontBackDist / 2.0f) {
					frontWallsDetected++;
				}
			}
			else if (fusedData.distSensorState.frontLeft == DistSensorStatus::underflow)
			{
				frontWallsDetected++;
			}

			if (fusedData.distSensorState.frontRight == DistSensorStatus::ok)
			{
				if (fusedData.distances.frontRight / 10.0f < 20.0f - SIALSettings::Mechanics::distSensFrontBackDist / 2.0f) {
					frontWallsDetected++;
				}
			}
			else if (fusedData.distSensorState.frontRight == DistSensorStatus::underflow)
			{
				frontWallsDetected++;
			}

			if (fusedData.distSensorState.leftFront == DistSensorStatus::ok)
			{
				leftWallsDetected++;
			}
			else if (fusedData.distSensorState.leftFront == DistSensorStatus::underflow)
			{
				leftWallsDetected++;
			}

			if (fusedData.distSensorState.leftBack == DistSensorStatus::ok)
			{
				leftWallsDetected++;
			}
			else if (fusedData.distSensorState.leftBack == DistSensorStatus::underflow)
			{
				leftWallsDetected++;
			}

			if (fusedData.distSensorState.rightFront == DistSensorStatus::ok)
			{
				rightWallsDetected++;
			}
			else if (fusedData.distSensorState.rightFront == DistSensorStatus::underflow)
			{
				rightWallsDetected++;
			}

			if (fusedData.distSensorState.rightBack == DistSensorStatus::ok)
			{
				rightWallsDetected++;
			}
			else if (fusedData.distSensorState.rightBack == DistSensorStatus::underflow)
			{
				rightWallsDetected++;
			}

			GridCell newCell;
			uint8_t sureWalls;
			static int consecutiveOk = 0;

			bool isOk = MazeMapping::manageDetectedWalls(frontWallsDetected, leftWallsDetected, rightWallsDetected, fusedData, newCell, sureWalls);
			fusedData.gridCell.cellConnections = newCell.cellConnections;

			if (isOk) {
				consecutiveOk++;
			}
			else {
				consecutiveOk = 0;
			}

			if (consecutiveOk >= 2) {
				Serial.println(leftWallsDetected);

				Serial.print("successful scan: ");
				Serial.print(sureWalls, BIN);
				Serial.print(", at: ");
				Serial.print(fusedData.robotState.mapCoordinate.x);
				Serial.print(", ");
				Serial.println(fusedData.robotState.mapCoordinate.y);

				fusedData.gridCell.cellConnections = (~sureWalls) & CellConnections::directionMask;
			}

			return consecutiveOk >= 2;
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

			if (fusedData.distSensorState.leftFront == DistSensorStatus::ok &&
				(fusedData.distances.leftFront / 10.0f < distToLWall + SIALSettings::MazeMapping::maxDistLongerThanBorder) ||
				fusedData.distSensorState.leftFront == DistSensorStatus::underflow) {
				static uint32_t lastRisingEdge = 0;
				if (fusedData.fusedDistSens.risingEdge.leftFront) {
					lastRisingEdge = millis();
				}
				else if (millis() - lastRisingEdge > 100) {
					usableData.lf = true;
					distances.lf = fusedData.distSensorState.leftFront == DistSensorStatus::underflow ? DistanceSensors::VL6180::minDist : fusedData.distances.leftFront;
				}
			}

			if (fusedData.distSensorState.leftBack == DistSensorStatus::ok &&
				(fusedData.distances.leftBack / 10.0f < distToLWall + SIALSettings::MazeMapping::maxDistLongerThanBorder) ||
				fusedData.distSensorState.leftBack == DistSensorStatus::underflow) {
				static uint32_t lastRisingEdge = 0;
				if (fusedData.fusedDistSens.risingEdge.leftBack) {
					lastRisingEdge = millis();
				}
				else if (millis() - lastRisingEdge > 100) {
					usableData.lb = true;
					distances.lb = fusedData.distSensorState.leftBack == DistSensorStatus::underflow ? DistanceSensors::VL6180::minDist : fusedData.distances.leftBack;
				}
			}

			if (fusedData.distSensorState.rightFront == DistSensorStatus::ok &&
				(fusedData.distances.rightFront / 10.0f < distToRWall + SIALSettings::MazeMapping::maxDistLongerThanBorder) ||
				fusedData.distSensorState.rightFront == DistSensorStatus::underflow) {
				static uint32_t lastRisingEdge = 0;
				if (fusedData.fusedDistSens.risingEdge.rightFront) {
					lastRisingEdge = millis();
				}
				else if (millis() - lastRisingEdge > 100) {
					usableData.rf = true;
					distances.rf = fusedData.distSensorState.rightFront == DistSensorStatus::underflow ? DistanceSensors::VL6180::minDist : fusedData.distances.rightFront;
				}
			}

			if (fusedData.distSensorState.rightBack == DistSensorStatus::ok &&
				(fusedData.distances.rightBack / 10.0f < distToRWall + SIALSettings::MazeMapping::maxDistLongerThanBorder) ||
				fusedData.distSensorState.rightBack == DistSensorStatus::underflow) {
				static uint32_t lastRisingEdge = 0;
				if (fusedData.fusedDistSens.risingEdge.rightBack) {
					lastRisingEdge = millis();
				}
				else if (millis() - lastRisingEdge > 100) {
					usableData.rb = true;
					distances.rb = fusedData.distSensorState.rightBack == DistSensorStatus::underflow ? DistanceSensors::VL6180::minDist : fusedData.distances.rightBack;
				}
			}

			if ((fusedData.distSensorState.frontLeft == DistSensorStatus::ok &&
				fusedData.distances.frontLeft < 300 - SIALSettings::Mechanics::distSensFrontBackDist * 10 * 0.9) ||
				fusedData.distSensorState.frontLeft == DistSensorStatus::underflow) {
				usableData.fl = true;
				distances.fl = fusedData.distSensorState.frontLeft == DistSensorStatus::underflow ? DistanceSensors::VL53L0::minDist : fusedData.distances.frontLeft;
			}

			if ((fusedData.distSensorState.frontRight == DistSensorStatus::ok &&
				fusedData.distances.frontRight < 300 - SIALSettings::Mechanics::distSensFrontBackDist * 10 * 0.9) ||
				fusedData.distSensorState.frontRight == DistSensorStatus::underflow) {
				usableData.fr = true;
				distances.fr = fusedData.distSensorState.frontRight == DistSensorStatus::underflow ? DistanceSensors::VL53L0::minDist : fusedData.distances.frontRight;
			}

			float angleL = 0.0f;
			float angleR = 0.0f;
			float angleF = 0.0f;

			float distToWallL = -1.0f;
			float distToWallR = -1.0f;
			float distToWallF = -1.0f;

			int numData = 0;

			if (SmoothDriving::getInformation().drivingStraight ||
				SmoothDriving::getInformation().finished ||
				SmoothDriving::getInformation().uid == DrivingTaskUID::alignWalls) {
				if (usableData.lf && usableData.lb) {
					calcAngleWallOffsetFromTwoDistances(&angleL, &distToWallL, distances.lf, distances.lb, SIALSettings::Mechanics::distSensLRSpacing, SIALSettings::Mechanics::distSensLeftRightDist);
					angleL *= -1.0f;

					if (fabsf(fitAngleToInterval((angleL - fusedData.robotState.globalHeading) * 4.0f) / 4.0f) > 10.0f * DEG_TO_RAD) {
						angleL = 0;
						distToWallL = -1;
					}
					else {
						numData++;
					}
				}

				if (usableData.rf && usableData.rb) {
					calcAngleWallOffsetFromTwoDistances(&angleR, &distToWallR, distances.rf, distances.rb, SIALSettings::Mechanics::distSensLRSpacing, SIALSettings::Mechanics::distSensLeftRightDist);

					if (fabsf(fitAngleToInterval((angleR - fusedData.robotState.globalHeading) * 4.0f) / 4.0f) > 10.0f * DEG_TO_RAD) {
						angleR = 0;
						distToWallR = -1;
					}
					else {
						numData++;
					}
				}

				if (usableData.fl && usableData.fr) {
					calcAngleWallOffsetFromTwoDistances(&angleF, &distToWallF, distances.fl, distances.fr, SIALSettings::Mechanics::distSensFrontSpacing, SIALSettings::Mechanics::distSensFrontBackDist);

					if (fabsf(fitAngleToInterval((angleF - fusedData.robotState.globalHeading) * 4.0f) / 4.0f) > 10.0f * DEG_TO_RAD) {
						angleF = 0;
						distToWallF = -1;
					}
					else {
						numData++;
					}
				}
			}

			fusedData.fusedDistSens.distToWalls.l = distToWallL;
			fusedData.fusedDistSens.distToWalls.r = distToWallR;
			fusedData.fusedDistSens.distToWalls.f = distToWallF;

			if (numData > 0) {
				float angle = (angleL + angleR + angleF) / numData;

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
				fusedData.fusedDistSens.distSensAngle = fitAngleToInterval(angle + wallAngle);
				fusedData.fusedDistSens.distSensAngleTrust = (numData + 2.0f) / 5.0f;
			}
			else {
				fusedData.fusedDistSens.distSensAngleTrust = 0.0f;
			}
		}

		void distSensFusion()
		{
			static bool firstEdgeDetection = true;
			static DistSensorStates lastStates;
			static Distances lastDistances;

			auto states = fusedData.distSensorState;
			auto distances = fusedData.distances;

			if (!(SmoothDriving::getInformation().drivingStraight &&
				fabsf(MotorControl::getSpeeds().left) >= SIALSettings::MotorControl::minSpeed &&
				fabsf(MotorControl::getSpeeds().right) >= SIALSettings::MotorControl::minSpeed) ||
				fusedData.gridCell.cellState & (CellState::ramp | CellState::stair)) {
				firstEdgeDetection = true;
			}
			else {
				if (firstEdgeDetection) {
					firstEdgeDetection = false;
				}
				else {
					float x = NAN;
					float y = NAN;

					float frontEdgeX = 0.0f;
					float backEdgeX = 0.0f;
					float frontEdgeY = 0.0f;
					float backEdgeY = 0.0f;

					switch (fusedData.robotState.heading)
					{
					case AbsoluteDir::north:
					{
						frontEdgeX = roundf((fusedData.robotState.position.x + SIALSettings::Mechanics::distSensLRSpacing / 2.0f - 15.0f) / 30.0f) * 30.0f + 15.0f;
						backEdgeX = roundf((fusedData.robotState.position.x - SIALSettings::Mechanics::distSensLRSpacing / 2.0f - 15.0f) / 30.0f) * 30.0f + 15.0f;
						break;
					}
					case AbsoluteDir::east:
					{
						frontEdgeY = roundf((fusedData.robotState.position.y - SIALSettings::Mechanics::distSensLRSpacing / 2.0f - 15.0f) / 30.0f) * 30.0f + 15.0f;
						backEdgeY = roundf((fusedData.robotState.position.y + SIALSettings::Mechanics::distSensLRSpacing / 2.0f - 15.0f) / 30.0f) * 30.0f + 15.0f;
						break;
					}
					case AbsoluteDir::south:
					{
						frontEdgeX = roundf((fusedData.robotState.position.x - SIALSettings::Mechanics::distSensLRSpacing / 2.0f - 15.0f) / 30.0f) * 30.0f + 15.0f;
						backEdgeX = roundf((fusedData.robotState.position.x + SIALSettings::Mechanics::distSensLRSpacing / 2.0f - 15.0f) / 30.0f) * 30.0f + 15.0f;
						break;
					}
					case AbsoluteDir::west:
					{
						frontEdgeY = roundf((fusedData.robotState.position.y + SIALSettings::Mechanics::distSensLRSpacing / 2.0f - 15.0f) / 30.0f) * 30.0f + 15.0f;
						backEdgeY = roundf((fusedData.robotState.position.y - SIALSettings::Mechanics::distSensLRSpacing / 2.0f - 15.0f) / 30.0f) * 30.0f + 15.0f;
						break;
					}
					default:
						break;
					}

					DistSensBool fallingEdge(false);
					DistSensBool risingEdge(false);

					if (lastStates.leftFront == DistSensorStatus::overflow &&
						(states.leftFront == DistSensorStatus::ok && distances.leftFront / 10.0f < distToLWall + SIALSettings::MazeMapping::maxDistLongerThanBorder) ||
						states.leftFront == DistSensorStatus::underflow) {
						risingEdge.leftFront = true;
					}

					if (lastStates.rightFront == DistSensorStatus::overflow &&
						(states.rightFront == DistSensorStatus::ok && distances.rightFront / 10.0f < distToRWall + SIALSettings::MazeMapping::maxDistLongerThanBorder) ||
						states.rightFront == DistSensorStatus::underflow) {
						risingEdge.rightFront = true;
					}

					if (lastStates.leftBack == DistSensorStatus::overflow &&
						(states.leftBack == DistSensorStatus::ok && distances.leftBack / 10.0f < distToLWall + SIALSettings::MazeMapping::maxDistLongerThanBorder) ||
						states.leftBack == DistSensorStatus::underflow) {
						risingEdge.leftBack = true;
					}

					if (lastStates.rightBack == DistSensorStatus::overflow &&
						(states.rightBack == DistSensorStatus::ok && distances.rightBack / 10.0f < distToRWall + SIALSettings::MazeMapping::maxDistLongerThanBorder) ||
						states.rightBack == DistSensorStatus::underflow) {
						risingEdge.rightBack = true;
					}

					if (states.leftFront == DistSensorStatus::overflow &&
						(lastStates.leftFront == DistSensorStatus::ok && lastDistances.leftFront / 10.0f < distToLWall + SIALSettings::MazeMapping::maxDistLongerThanBorder) ||
						lastStates.leftFront == DistSensorStatus::underflow) {
						fallingEdge.leftFront = true;
					}

					if (states.rightFront == DistSensorStatus::overflow &&
						(lastStates.rightFront == DistSensorStatus::ok && lastDistances.rightFront / 10.0f < distToRWall + SIALSettings::MazeMapping::maxDistLongerThanBorder) ||
						lastStates.rightFront == DistSensorStatus::underflow) {
						fallingEdge.rightFront = true;
					}

					if (states.leftBack == DistSensorStatus::overflow &&
						(lastStates.leftBack == DistSensorStatus::ok && lastDistances.leftBack / 10.0f < distToLWall + SIALSettings::MazeMapping::maxDistLongerThanBorder) ||
						lastStates.leftBack == DistSensorStatus::underflow) {
						fallingEdge.leftBack = true;
					}

					if (states.rightBack == DistSensorStatus::overflow &&
						(lastStates.rightBack == DistSensorStatus::ok && lastDistances.rightBack / 10.0f < distToRWall + SIALSettings::MazeMapping::maxDistLongerThanBorder) ||
						lastStates.rightBack == DistSensorStatus::underflow) {
						fallingEdge.rightBack = true;
					}

					float weight = 0.4f;

					if (risingEdge.leftFront || risingEdge.rightFront) {
						if (risingEdge.leftFront && risingEdge.rightFront) {
							weight = 0.6f;
						}

						// front rising edge
						//Serial.println("front rising edge");

						switch (fusedData.robotState.heading)
						{
						case AbsoluteDir::north:
						{
							x = frontEdgeX - (SIALSettings::Mechanics::distSensLRSpacing / 2.0f + (fusedData.robotState.forwardVel > 0.0f ? 1.0f : -1.0f));
							break;
						}
						case AbsoluteDir::east:
						{
							y = frontEdgeY + (SIALSettings::Mechanics::distSensLRSpacing / 2.0f + (fusedData.robotState.forwardVel > 0.0f ? 1.0f : -1.0f));
							break;
						}
						case AbsoluteDir::south:
						{
							x = frontEdgeX + (SIALSettings::Mechanics::distSensLRSpacing / 2.0f + (fusedData.robotState.forwardVel > 0.0f ? 1.0f : -1.0f));
							break;
						}
						case AbsoluteDir::west:
						{
							y = frontEdgeY - (SIALSettings::Mechanics::distSensLRSpacing / 2.0f + (fusedData.robotState.forwardVel > 0.0f ? 1.0f : -1.0f));
							break;
						}
						default:
							break;
						}
					}
					else if (fallingEdge.leftFront || fallingEdge.rightFront) {
						if (fallingEdge.leftFront || fallingEdge.rightFront) {
							weight = 0.6f;
						}

						// front falling edge
						//Serial.println("front falling edge");

						switch (fusedData.robotState.heading)
						{
						case AbsoluteDir::north:
						{
							x = frontEdgeX - (SIALSettings::Mechanics::distSensLRSpacing / 2.0f - (fusedData.robotState.forwardVel > 0.0f ? 1.0f : -1.0f));
							break;
						}
						case AbsoluteDir::east:
						{
							y = frontEdgeY + (SIALSettings::Mechanics::distSensLRSpacing / 2.0f - (fusedData.robotState.forwardVel > 0.0f ? 1.0f : -1.0f));
							break;
						}
						case AbsoluteDir::south:
						{
							x = frontEdgeX + (SIALSettings::Mechanics::distSensLRSpacing / 2.0f - (fusedData.robotState.forwardVel > 0.0f ? 1.0f : -1.0f));
							break;
						}
						case AbsoluteDir::west:
						{
							y = frontEdgeY - (SIALSettings::Mechanics::distSensLRSpacing / 2.0f - (fusedData.robotState.forwardVel > 0.0f ? 1.0f : -1.0f));
							break;
						}
						default:
							break;
						}
					}

					if (risingEdge.leftBack || risingEdge.rightBack) {
						if (risingEdge.leftBack || risingEdge.rightBack) {
							weight = 0.6f;
						}

						// back rising edge
						//Serial.println("back rising edge");

						switch (fusedData.robotState.heading)
						{
						case AbsoluteDir::north:
						{
							x = backEdgeX + (SIALSettings::Mechanics::distSensLRSpacing / 2.0f - (fusedData.robotState.forwardVel > 0.0f ? 1.0f : -1.0f));
							break;
						}
						case AbsoluteDir::east:
						{
							y = backEdgeY - (SIALSettings::Mechanics::distSensLRSpacing / 2.0f - (fusedData.robotState.forwardVel > 0.0f ? 1.0f : -1.0f));
							break;
						}
						case AbsoluteDir::south:
						{
							x = backEdgeX - (SIALSettings::Mechanics::distSensLRSpacing / 2.0f - (fusedData.robotState.forwardVel > 0.0f ? 1.0f : -1.0f));
							break;
						}
						case AbsoluteDir::west:
						{
							y = backEdgeY + (SIALSettings::Mechanics::distSensLRSpacing / 2.0f - (fusedData.robotState.forwardVel > 0.0f ? 1.0f : -1.0f));
							break;
						}
						default:
							break;
						}
					}
					else if (fallingEdge.leftBack || fallingEdge.rightBack) {
						if (fallingEdge.leftBack || fallingEdge.rightBack) {
							weight = 0.6f;
						}

						// back falling edge
						//Serial.println("back falling edge");

						switch (fusedData.robotState.heading)
						{
						case AbsoluteDir::north:
						{
							x = backEdgeX + (SIALSettings::Mechanics::distSensLRSpacing / 2.0f + (fusedData.robotState.forwardVel > 0.0f ? 1.0f : -1.0f));
							break;
						}
						case AbsoluteDir::east:
						{
							y = backEdgeY - (SIALSettings::Mechanics::distSensLRSpacing / 2.0f + (fusedData.robotState.forwardVel > 0.0f ? 1.0f : -1.0f));
							break;
						}
						case AbsoluteDir::south:
						{
							x = backEdgeX - (SIALSettings::Mechanics::distSensLRSpacing / 2.0f + (fusedData.robotState.forwardVel > 0.0f ? 1.0f : -1.0f));
							break;
						}
						case AbsoluteDir::west:
						{
							y = backEdgeY + (SIALSettings::Mechanics::distSensLRSpacing / 2.0f + (fusedData.robotState.forwardVel > 0.0f ? 1.0f : -1.0f));
							break;
						}
						default:
							break;
						}
					}

					fusedData.fusedDistSens.fallingEdge = fallingEdge;
					fusedData.fusedDistSens.risingEdge = risingEdge;

					if (!std::isnan(x)) {
						//Serial.print("new edge x: ");
						//Serial.print(x);
						//Serial.print(", pos.x: ");
						//Serial.println(fusedData.robotState.position.x);

						// Adjust position for delay 50ms in driving direction
						//x += 0.05f * fusedData.robotState.forwardVel * sgn(cosf(fusedData.robotState.globalHeading));

						// TESTING
						//correctedState.x = x * weight + fusedData.robotState.position.x * (1.0f - weight);
						//correctedState.newX = true;
					}
					else if (!std::isnan(y)) {
						//Serial.print("new edge y: ");
						//Serial.print(y);
						//Serial.print(", pos.y: ");
						//Serial.println(fusedData.robotState.position.y);

						// Adjust position for delay 50ms in driving direction
						//x += 0.05f * fusedData.robotState.forwardVel * sgn(sinf(fusedData.robotState.globalHeading));

						// TESTING
						//correctedState.y = y * weight + fusedData.robotState.position.y * (1.0f - weight);
						//correctedState.newY = true;
					}
				}
			}

			calcOffsetAngleFromDistSens();

			if (forceAnglePosReset) {
				forceAnglePosReset = false;
				updatePosAndRotFromDist(300);
			}

			lastStates = states;
			lastDistances = distances;
		}

		void updatePosAndRotFromDist(uint32_t time) {
			float totAngleWeight = 0.0f;
			float avgCos = 0.0f;
			float avgSin = 0.0f;
			float avgX = 0.0f;
			int totXNum = 0.0f;
			float avgY = 0.0f;
			int totYNum = 0.0f;

			const RobotState robotState = fusedData.robotState;

			const auto start = millis();
			while (millis() - start < time) {
				SensorFusion::updateSensors();
				SensorFusion::distSensFusion();
				SensorFusion::sensorFusion();
				calcOffsetAngleFromDistSens();

				if (fusedData.fusedDistSens.distSensAngleTrust > 0.01f) {
					totAngleWeight += fusedData.fusedDistSens.distSensAngleTrust;
					avgCos += cosf(fusedData.fusedDistSens.distSensAngle) * fusedData.fusedDistSens.distSensAngleTrust;
					avgSin += sinf(fusedData.fusedDistSens.distSensAngle) * fusedData.fusedDistSens.distSensAngleTrust;
				}

				switch (robotState.heading)
				{
				case AbsoluteDir::north:
					if (fusedData.fusedDistSens.distToWalls.l > 0.0f) {
						avgY += (robotState.mapCoordinate.y + 0.5f) * 30.0f - fusedData.fusedDistSens.distToWalls.l;
						totYNum++;
					}

					if (fusedData.fusedDistSens.distToWalls.r > 0.0f) {
						avgY += (robotState.mapCoordinate.y - 0.5f) * 30.0f + fusedData.fusedDistSens.distToWalls.r;
						totYNum++;
					}

					if (fusedData.fusedDistSens.distToWalls.f > 0.0f) {
						avgX += (robotState.mapCoordinate.x + 0.5f) * 30.0f - fusedData.fusedDistSens.distToWalls.f;
						totXNum++;
					}
					break;
				case AbsoluteDir::east:
					if (fusedData.fusedDistSens.distToWalls.l > 0.0f) {
						avgX += (robotState.mapCoordinate.x + 0.5f) * 30.0f - fusedData.fusedDistSens.distToWalls.l;
						totXNum++;
					}

					if (fusedData.fusedDistSens.distToWalls.r > 0.0f) {
						avgX += (robotState.mapCoordinate.x - 0.5f) * 30.0f + fusedData.fusedDistSens.distToWalls.r;
						totXNum++;
					}

					if (fusedData.fusedDistSens.distToWalls.f > 0.0f) {
						avgY += (robotState.mapCoordinate.y - 0.5f) * 30.0f + fusedData.fusedDistSens.distToWalls.f;
						totYNum++;
					}
					break;
				case AbsoluteDir::south:
					if (fusedData.fusedDistSens.distToWalls.l > 0.0f) {
						avgY += (robotState.mapCoordinate.y - 0.5f) * 30.0f + fusedData.fusedDistSens.distToWalls.l;
						totYNum++;
					}

					if (fusedData.fusedDistSens.distToWalls.r > 0.0f) {
						avgY += (robotState.mapCoordinate.y + 0.5f) * 30.0f - fusedData.fusedDistSens.distToWalls.r;
						totYNum++;
					}

					if (fusedData.fusedDistSens.distToWalls.f > 0.0f) {
						avgX += (robotState.mapCoordinate.x - 0.5f) * 30.0f + fusedData.fusedDistSens.distToWalls.f;
						totXNum++;
					}
					break;
				case AbsoluteDir::west:
					if (fusedData.fusedDistSens.distToWalls.l > 0.0f) {
						avgX += (robotState.mapCoordinate.x - 0.5f) * 30.0f + fusedData.fusedDistSens.distToWalls.l;
						totXNum++;
					}

					if (fusedData.fusedDistSens.distToWalls.r > 0.0f) {
						avgX += (robotState.mapCoordinate.x + 0.5f) * 30.0f - fusedData.fusedDistSens.distToWalls.r;
						totXNum++;
					}

					if (fusedData.fusedDistSens.distToWalls.f > 0.0f) {
						avgY += (robotState.mapCoordinate.y + 0.5f) * 30.0f - fusedData.fusedDistSens.distToWalls.f;
						totYNum++;
					}
					break;
				default:
					break;
				}
			}

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

				correctedState.heading = makeRotationCoherent(robotState.globalHeading, avgAngle);
				correctedState.newHeading = true;

				float currentRotEncAngle = (MotorControl::getDistance(Motor::right) - MotorControl::getDistance(Motor::left)) / (SIALSettings::Mechanics::wheelDistToMiddle * 2.0f * SIALSettings::SensorFusion::chi);
				totalHeadingOff = fitAngleToInterval(currentRotEncAngle - avgAngle);

				Gyro::tare(avgAngle);
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
			DistanceSensors::updateDistSensors();
			Gyro::updateValues();
		}

		void setDistances(Distances distances)
		{
			fusedData.distances = distances;
		}

		void setDistSensStates(DistSensorStates distSensorStates)
		{
			fusedData.distSensorState = distSensorStates;
		}

		void setUpdatedDistSens(DistSensBool distSensUpdates) {
			fusedData.distSensUpdates = distSensUpdates;
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

			float angle = fusedData.fusedDistSens.distSensAngleTrust > 0.01f ? fusedData.fusedDistSens.distSensAngle : NAN;
			return fitAngleToInterval(angle - wallAngle);
		}
	}
}