/*
This is the heart of the robot
*/

#include "../header/RobotLogic.h"
#include "../header/SensorFusion.h"
#include "../header/MazeMapping.h"
#include "../header/SmoothDriving.h"
#include "../header/MotorControl.h"
#include "../header/HeatSensor.h"
#include "../header/Dispenser.h"
#include "../header/CamRec.h"

namespace JAFD
{
	namespace RobotLogic
	{
		namespace {
			void updateNextMapPosHeading(AbsoluteDir absDir, MapCoordinate& mapCoordinate, AbsoluteDir& heading) {
				heading = absDir;

				switch (absDir)
				{
				case AbsoluteDir::north:
					mapCoordinate.x += 1;
					break;
				case AbsoluteDir::east:
					mapCoordinate.y -= 1;
					break;
				case AbsoluteDir::south:
					mapCoordinate.x -= 1;
					break;
				case AbsoluteDir::west:
					mapCoordinate.y += 1;
					break;
				default:
					break;
				}
			}

			void startRelativeTurnDirDrive(RelativeDir dir) {
				using namespace SmoothDriving;

				switch (dir)
				{
				case RelativeDir::forward:
					setNewTask<NewStateType::currentState>(TaskArray(Stop(),
						FollowWall(20, 30.0f),
						Stop()));
					break;
				case RelativeDir::right:
					setNewTask<NewStateType::currentState>(TaskArray(Stop(),
						Rotate(-0.5f, -90.0f),
						FollowWall(20, 30.0f),
						Stop()));
					break;
				case RelativeDir::backward:
					setNewTask<NewStateType::currentState>(TaskArray(Stop(),
						Rotate(0.5f, 180.0f),
						FollowWall(20, 30.0f),
						Stop()));
					break;
				case RelativeDir::left:
					setNewTask<NewStateType::currentState>(TaskArray(Stop(),
						Rotate(0.5f, 90.0f),
						FollowWall(20, 30.0f),
						Stop()));
					break;
				default:
					break;
				}
			}
		}

		// Start task which prefers straigth over left over right over back
		// Intended for the first maneuver after starting
		void startFirstTask(FusedData tempFusedData) {
			AbsoluteDir absLeft = makeAbsolute(RelativeDir::left, tempFusedData.robotState.heading);
			AbsoluteDir absForward = makeAbsolute(RelativeDir::forward, tempFusedData.robotState.heading);
			AbsoluteDir absRight = makeAbsolute(RelativeDir::right, tempFusedData.robotState.heading);
			AbsoluteDir absBack = makeAbsolute(RelativeDir::backward, tempFusedData.robotState.heading);

			if (tempFusedData.gridCell.cellConnections & static_cast<uint8_t>(absForward)) {
				startRelativeTurnDirDrive(RelativeDir::forward);
			}
			else
			{
				if (tempFusedData.gridCell.cellConnections & static_cast<uint8_t>(absLeft) && tempFusedData.gridCell.cellConnections & static_cast<uint8_t>(absRight))
				{
					if (random(2) == 0) {
						startRelativeTurnDirDrive(RelativeDir::left);
					}
					else {
						startRelativeTurnDirDrive(RelativeDir::right);
					}
				}
				else if (tempFusedData.gridCell.cellConnections & static_cast<uint8_t>(absLeft)) {
					startRelativeTurnDirDrive(RelativeDir::left);
				}
				else if (tempFusedData.gridCell.cellConnections & static_cast<uint8_t>(absRight)) {
					startRelativeTurnDirDrive(RelativeDir::right);
				}
				else if (tempFusedData.gridCell.cellConnections & static_cast<uint8_t>(absBack)) {
					startRelativeTurnDirDrive(RelativeDir::backward);
				}
				else {
					Serial.println("No direction to drive!");
					return;
				}
			}
		}

		void loop()
		{
			using namespace SmoothDriving;

			static bool start = true;
			static bool startAfterRotation = false;
			static MapCoordinate nextMapCoordinate;
			static AbsoluteDir nextHeading;

			const auto tempFusedData = SensorFusion::getFusedData();

			if (start) {
				if (!SensorFusion::scanSurrounding()) {
					return;
				}

				SensorFusion::updatePosAndRotFromDist();

				if (tempFusedData.gridCell.cellConnections & EntranceDirections::east) {
					setNewTask<NewStateType::currentState>(TaskArray(Stop(),
						Rotate(-0.5f, 90.0f),
						Stop()));
					startAfterRotation = true;
				}
				else {
					setNewTask<NewStateType::currentState>(TaskArray(Stop(),
						Rotate(0.5f, 90.0f),
						Stop()));
					startAfterRotation = true;
				}

				start = false;
			}
			else if (startAfterRotation) {
				if (!SmoothDriving::isTaskFinished()) {
					return;
				}

				if (!SensorFusion::scanSurrounding()) {
					return;
				}

				SensorFusion::updatePosAndRotFromDist();

				startAfterRotation = false;

				startFirstTask(tempFusedData);
			}
			else {
				if (SmoothDriving::isTaskFinished()) {
					if (SensorFusion::scanSurrounding()) {
						SensorFusion::updatePosAndRotFromDist();
						GridCell cell;
						MazeMapping::getGridCell(&cell, tempFusedData.robotState.mapCoordinate);
						startFirstTask(tempFusedData);
					}
				}

				static int consecutiveLHeat = 0;
				static int consecutiveRHeat = 0;
				static MapCoordinate lastFoundVictim = homePosition;

				if (HeatSensor::detectVictim(HeatSensorSide::left) &&
					!(tempFusedData.gridCell.cellConnections & static_cast<uint8_t>(makeAbsolute(RelativeDir::left, tempFusedData.robotState.heading)))) {
					consecutiveLHeat++;
				}
				else {
					consecutiveLHeat = 0;
				}

				if (HeatSensor::detectVictim(HeatSensorSide::right) &&
					!(tempFusedData.gridCell.cellConnections & static_cast<uint8_t>(makeAbsolute(RelativeDir::right, tempFusedData.robotState.heading)))) {
					consecutiveRHeat++;
				}
				else {
					consecutiveRHeat = 0;
				}

				if (lastFoundVictim != tempFusedData.robotState.mapCoordinate) {
					if (consecutiveLHeat > 5) {
						SmoothDriving::stopTask();
						Dispenser::dispenseLeft(3);
						lastFoundVictim = tempFusedData.robotState.mapCoordinate;
					}

					if (consecutiveRHeat > 5) {
						SmoothDriving::stopTask();
						Dispenser::dispenseLeft(3);
						lastFoundVictim = tempFusedData.robotState.mapCoordinate;
					}
				}
			}
		}

		void timeBetweenUpdate()
		{
			// TODO
			// CamRec::loop();
		}
	}
}