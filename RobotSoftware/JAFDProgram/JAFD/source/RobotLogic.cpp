/*
This is the heart of the robot
*/

#include "../header/RobotLogic.h"
#include "../header/SensorFusion.h"
#include "../header/MazeMapping.h"
#include "../header/SmoothDriving.h"
#include "../header/MotorControl.h"
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
						FollowWall(25, 30.0f),
						Stop()));
					break;
				case RelativeDir::right:
					setNewTask<NewStateType::currentState>(TaskArray(Stop(),
						Rotate(-1.0f, -90.0f),
						FollowWall(25, 30.0f),
						Stop()));
					break;
				case RelativeDir::backward:
					setNewTask<NewStateType::currentState>(TaskArray(Stop(),
						Rotate(1.0f, 180.0f),
						FollowWall(25, 30.0f),	
						Stop()));
					break;
				case RelativeDir::left:
					setNewTask<NewStateType::currentState>(TaskArray(Stop(),
						Rotate(1.0f, 90.0f),
						FollowWall(25, 30.0f),
						Stop()));
					break;
				default:
					break;
				}
			}
		}

		void loop()
		{
			using namespace SmoothDriving;

			static bool start = true;
			static MapCoordinate nextMapCoordinate;
			static AbsoluteDir nextHeading;

			//static const TaskArray tasks[] = {
			//	TaskArray(SmoothDriving::Stop(),
			//			SmoothDriving::FollowWall(25, 30.0f),
			//			SmoothDriving::Stop()),
			//	TaskArray(SmoothDriving::Stop(),
			//			SmoothDriving::FollowWall(25, 60.0f),
			//			SmoothDriving::Stop()),
			//	TaskArray(SmoothDriving::Stop(),
			//			SmoothDriving::Rotate(-0.5f, -90.0f),
			//			SmoothDriving::Stop()),
			//	//TaskArray(SmoothDriving::Stop(),
			//	//		SmoothDriving::FollowWall(25, 30.0f),
			//	//		SmoothDriving::Stop()),
			//	//TaskArray(SmoothDriving::Stop(),
			//	//		SmoothDriving::FollowWall(25, 60.0f),
			//	//		SmoothDriving::Stop())
			//};

			//const static uint16_t numTasks = sizeof(tasks) / sizeof(*tasks);

			//static uint16_t i = 0;

			//if (isTaskFinished() && i < numTasks)
			//{
			//	setNewTask<NewStateType::lastEndState>(tasks[i]);
			//	i++;
			//	// i %= numTasks;
			//}

			//return;

			if (start) {

				if (!SensorFusion::scanSurrounding()) {
					return;
				}

				const auto tempFusedData = SensorFusion::getFusedData();

				if (tempFusedData.gridCell.cellConnections & EntranceDirections::north) {
					startRelativeTurnDirDrive(RelativeDir::forward);
					updateNextMapPosHeading(AbsoluteDir::north, nextMapCoordinate, nextHeading);
				}
				else if (tempFusedData.gridCell.cellConnections & EntranceDirections::east) {
					startRelativeTurnDirDrive(RelativeDir::right);
					updateNextMapPosHeading(AbsoluteDir::east, nextMapCoordinate, nextHeading);
				}
				else if (tempFusedData.gridCell.cellConnections & EntranceDirections::west) {
					startRelativeTurnDirDrive(RelativeDir::left);
					updateNextMapPosHeading(AbsoluteDir::west, nextMapCoordinate, nextHeading);
				}
				else {
					startRelativeTurnDirDrive(RelativeDir::backward);
					updateNextMapPosHeading(AbsoluteDir::south, nextMapCoordinate, nextHeading);
				}

				start = false;
			}

			if (SmoothDriving::isTaskFinished())
			{
				SensorFusion::setMapPosHeading(nextMapCoordinate, nextHeading);

				if (!SensorFusion::scanSurrounding()) {
					return;
				}

				const auto tempFusedData = SensorFusion::getFusedData();

				// left wall following
				AbsoluteDir absLeft = makeAbsolute(RelativeDir::left, tempFusedData.robotState.heading);
				AbsoluteDir absForward = makeAbsolute(RelativeDir::forward, tempFusedData.robotState.heading);
				AbsoluteDir absRight = makeAbsolute(RelativeDir::right, tempFusedData.robotState.heading);
				AbsoluteDir absBack = makeAbsolute(RelativeDir::backward, tempFusedData.robotState.heading);

				if (tempFusedData.gridCell.cellConnections & static_cast<uint8_t>(absLeft)) {
					startRelativeTurnDirDrive(RelativeDir::left);
					updateNextMapPosHeading(absLeft, nextMapCoordinate, nextHeading);
				}
				else if (tempFusedData.gridCell.cellConnections & static_cast<uint8_t>(absForward)) {
					startRelativeTurnDirDrive(RelativeDir::forward);
					updateNextMapPosHeading(absForward, nextMapCoordinate, nextHeading);
				}
				else if (tempFusedData.gridCell.cellConnections & static_cast<uint8_t>(absRight)) {
					startRelativeTurnDirDrive(RelativeDir::right);
					updateNextMapPosHeading(absRight, nextMapCoordinate, nextHeading);
				}
				else if (tempFusedData.gridCell.cellConnections & static_cast<uint8_t>(absBack)) {
					startRelativeTurnDirDrive(RelativeDir::backward);
					updateNextMapPosHeading(absBack, nextMapCoordinate, nextHeading);
				}
				else {
					Serial.println("No direction to drive!");
					return;
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