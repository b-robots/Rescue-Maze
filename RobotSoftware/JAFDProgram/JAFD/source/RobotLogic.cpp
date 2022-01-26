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
			void startRelativeTurnDirDrive(RelativeDir dir) {
				switch (dir)
				{
				case RelativeDir::forward:
					SmoothDriving::setNewTask<SmoothDriving::NewStateType::lastEndState>(SmoothDriving::TaskArray(SmoothDriving::Stop(),
						SmoothDriving::FollowWall(20, 30.0f),
						SmoothDriving::Stop()));
					break;
				case RelativeDir::right:
					SmoothDriving::setNewTask<SmoothDriving::NewStateType::lastEndState>(SmoothDriving::TaskArray(SmoothDriving::Stop(),
						SmoothDriving::Rotate(-2.0f, -90.0f),
						SmoothDriving::FollowWall(20, 30.0f),
						SmoothDriving::Stop()));
					break;
				case RelativeDir::backward:
					SmoothDriving::setNewTask<SmoothDriving::NewStateType::lastEndState>(SmoothDriving::TaskArray(SmoothDriving::Stop(),
						SmoothDriving::Rotate(3.0f, 180.0f),
						SmoothDriving::FollowWall(20, 30.0f),
						SmoothDriving::Stop()));
					break;
				case RelativeDir::left:
					SmoothDriving::setNewTask<SmoothDriving::NewStateType::lastEndState>(SmoothDriving::TaskArray(SmoothDriving::Stop(),
						SmoothDriving::Rotate(2.0f, 90.0f),
						SmoothDriving::FollowWall(20, 30.0f),
						SmoothDriving::Stop()));
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

			const auto tempFusedData = SensorFusion::getFusedData();

			static const TaskArray tasks[] = {
				ForceSpeed(15, 50.0f),
				Rotate(2.0f, -90.0f)
			};

			const static uint16_t numTasks = sizeof(tasks) / sizeof(*tasks);

			static uint16_t i = 0;

			if (isTaskFinished() && i < numTasks)
			{
				setNewTask<NewStateType::currentState>(tasks[i]);
				i++;
				i %= numTasks;
			}

			return;

			if (start) {
				if (tempFusedData.gridCellCertainty >= 0.5f) {
					if (tempFusedData.gridCell.cellConnections && EntranceDirections::north) {
						startRelativeTurnDirDrive(RelativeDir::forward);
					}
					else if (tempFusedData.gridCell.cellConnections && EntranceDirections::east) {
						startRelativeTurnDirDrive(RelativeDir::right);
					}
					else if (tempFusedData.gridCell.cellConnections && EntranceDirections::west) {
						startRelativeTurnDirDrive(RelativeDir::left);
					}
					else if (tempFusedData.gridCell.cellConnections && EntranceDirections::south) {
						startRelativeTurnDirDrive(RelativeDir::backward);
					}
					else {
						Serial.println("No direction to drive from starting position!");
						return;
					}

					start = false;
				}
			}

			if (SmoothDriving::isTaskFinished() && tempFusedData.gridCellCertainty >= 0.5f)
			{
				// left wall following
				if (tempFusedData.gridCell.cellConnections && static_cast<uint8_t>(makeAbsolute(RelativeDir::left, tempFusedData.robotState.heading))) {
					startRelativeTurnDirDrive(RelativeDir::left);
				}
				else if (tempFusedData.gridCell.cellConnections && static_cast<uint8_t>(makeAbsolute(RelativeDir::forward, tempFusedData.robotState.heading))) {
					startRelativeTurnDirDrive(RelativeDir::forward);
				}
				else if (tempFusedData.gridCell.cellConnections && static_cast<uint8_t>(makeAbsolute(RelativeDir::right, tempFusedData.robotState.heading))) {
					startRelativeTurnDirDrive(RelativeDir::right);
				}
				else if (tempFusedData.gridCell.cellConnections && static_cast<uint8_t>(makeAbsolute(RelativeDir::backward, tempFusedData.robotState.heading))) {
					startRelativeTurnDirDrive(RelativeDir::backward);
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