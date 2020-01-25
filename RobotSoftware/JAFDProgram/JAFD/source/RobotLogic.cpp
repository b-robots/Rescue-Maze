/*
This is the heart of the robot
*/

#include "../header/RobotLogic.h"
#include "../header/SensorFusion.h"
#include "../header/MazeMapping.h"
#include "../header/SmoothDriving.h"
#include "../header/MotorControl.h"

namespace JAFD
{
	namespace RobotLogic
	{
		void loop()
		{
			if (SensorFusion::getFusedData().gridCellCertainty >= 0.5f)
			{
				SmoothDriving::TaskArray test = SmoothDriving::TaskArray(SmoothDriving::Stop(), SmoothDriving::Accelerate(), SmoothDriving::DriveStraight(), SmoothDriving::Accelerate(), SmoothDriving::Stop());

				test.updateSpeeds(0);

				GridCell cell = SensorFusion::getFusedData().gridCell;

				if (SmoothDriving::isTaskFinished())
				{
					if (cell.cellConnections & Direction::north)
					{
						switch (SensorFusion::getFusedData().heading)
						{
						case HeadingDirection::north:
							SmoothDriving::setNewTask<SmoothDriving::NewStateType::lastEndState>(SmoothDriving::TaskArray(	SmoothDriving::Stop(),
																															SmoothDriving::Accelerate(30, 15.0f),
																															SmoothDriving::Accelerate(0, 15.0f),
																															SmoothDriving::Stop()));
							break;
						case HeadingDirection::east:
							SmoothDriving::setNewTask<SmoothDriving::NewStateType::lastEndState>(SmoothDriving::TaskArray(	SmoothDriving::Stop(),
																															SmoothDriving::Rotate(5, 90.0f),
																															SmoothDriving::Accelerate(30, 15.0f),
																															SmoothDriving::Accelerate(0, 15.0f),
																															SmoothDriving::Stop()));
							break;
						case HeadingDirection::south:
							SmoothDriving::setNewTask<SmoothDriving::NewStateType::lastEndState>(SmoothDriving::TaskArray(	SmoothDriving::Stop(),
																															SmoothDriving::Rotate(5, 180.0f),
																															SmoothDriving::Accelerate(30, 15.0f),
																															SmoothDriving::Accelerate(0, 15.0f),
																															SmoothDriving::Stop()));
							break;
						case HeadingDirection::west:
							SmoothDriving::setNewTask<SmoothDriving::NewStateType::lastEndState>(SmoothDriving::TaskArray(	SmoothDriving::Stop(),
																															SmoothDriving::Rotate(-5, -90.0f),
																															SmoothDriving::Accelerate(30, 15.0f),
																															SmoothDriving::Accelerate(0, 15.0f),
																															SmoothDriving::Stop()));
							break;
						default:
							break;
						}
					}
					else if (cell.cellConnections & Direction::east)
					{
						switch (SensorFusion::getFusedData().heading)
						{
						case HeadingDirection::north:
							SmoothDriving::setNewTask<SmoothDriving::NewStateType::lastEndState>(SmoothDriving::Stop());
							while (!SmoothDriving::isTaskFinished());
							SmoothDriving::setNewTask<SmoothDriving::NewStateType::lastEndState>(SmoothDriving::Rotate(-5, -90.0f));
							while (!SmoothDriving::isTaskFinished());
							SmoothDriving::setNewTask<SmoothDriving::NewStateType::lastEndState>(SmoothDriving::Accelerate(30, 30.0f));
							break;
						case HeadingDirection::east:
							SmoothDriving::setNewTask<SmoothDriving::NewStateType::lastEndState>(SmoothDriving::Stop());
							while (!SmoothDriving::isTaskFinished());
							SmoothDriving::setNewTask<SmoothDriving::NewStateType::lastEndState>(SmoothDriving::Accelerate(30, 30.0f));
							break;
						case HeadingDirection::south:
							SmoothDriving::setNewTask<SmoothDriving::NewStateType::lastEndState>(SmoothDriving::Stop());
							while (!SmoothDriving::isTaskFinished());
							SmoothDriving::setNewTask<SmoothDriving::NewStateType::lastEndState>(SmoothDriving::Rotate(5, 90.0f));
							while (!SmoothDriving::isTaskFinished());
							SmoothDriving::setNewTask<SmoothDriving::NewStateType::lastEndState>(SmoothDriving::Accelerate(30, 30.0f));
							break;
						case HeadingDirection::west:
							SmoothDriving::setNewTask<SmoothDriving::NewStateType::lastEndState>(SmoothDriving::Stop());
							while (!SmoothDriving::isTaskFinished());
							SmoothDriving::setNewTask<SmoothDriving::NewStateType::lastEndState>(SmoothDriving::Rotate(5, 180.0f));
							while (!SmoothDriving::isTaskFinished());
							SmoothDriving::setNewTask<SmoothDriving::NewStateType::lastEndState>(SmoothDriving::Accelerate(30, 30.0f));
							break;
						default:
							break;
						}
					}
					else if (cell.cellConnections & Direction::south)
					{
						switch (SensorFusion::getFusedData().heading)
						{
						case HeadingDirection::north:
							SmoothDriving::setNewTask<SmoothDriving::NewStateType::lastEndState>(SmoothDriving::Stop());
							while (!SmoothDriving::isTaskFinished());
							SmoothDriving::setNewTask<SmoothDriving::NewStateType::lastEndState>(SmoothDriving::Rotate(5, 180.0f));
							while (!SmoothDriving::isTaskFinished());
							SmoothDriving::setNewTask<SmoothDriving::NewStateType::lastEndState>(SmoothDriving::Accelerate(30, 30.0f));
							break;
						case HeadingDirection::east:
							SmoothDriving::setNewTask<SmoothDriving::NewStateType::lastEndState>(SmoothDriving::Stop());
							while (!SmoothDriving::isTaskFinished());
							SmoothDriving::setNewTask<SmoothDriving::NewStateType::lastEndState>(SmoothDriving::Rotate(-5, -90.0f));
							while (!SmoothDriving::isTaskFinished());
							SmoothDriving::setNewTask<SmoothDriving::NewStateType::lastEndState>(SmoothDriving::Accelerate(30, 30.0f));
							break;
						case HeadingDirection::south:
							SmoothDriving::setNewTask<SmoothDriving::NewStateType::lastEndState>(SmoothDriving::Stop());
							while (!SmoothDriving::isTaskFinished());
							SmoothDriving::setNewTask<SmoothDriving::NewStateType::lastEndState>(SmoothDriving::Accelerate(30, 30.0f));
							break;
						case HeadingDirection::west:
							SmoothDriving::setNewTask<SmoothDriving::NewStateType::lastEndState>(SmoothDriving::Stop());
							while (!SmoothDriving::isTaskFinished());
							SmoothDriving::setNewTask<SmoothDriving::NewStateType::lastEndState>(SmoothDriving::Rotate(5, 90.0f));
							while (!SmoothDriving::isTaskFinished());
							SmoothDriving::setNewTask<SmoothDriving::NewStateType::lastEndState>(SmoothDriving::Accelerate(30, 30.0f));
							break;
						default:
							break;
						}
					}
					else if (cell.cellConnections & Direction::west)
					{
						switch (SensorFusion::getFusedData().heading)
						{
						case HeadingDirection::north:
							SmoothDriving::setNewTask<SmoothDriving::NewStateType::lastEndState>(SmoothDriving::Stop());
							while (!SmoothDriving::isTaskFinished());
							SmoothDriving::setNewTask<SmoothDriving::NewStateType::lastEndState>(SmoothDriving::Rotate(5, 90.0f));
							while (!SmoothDriving::isTaskFinished());
							SmoothDriving::setNewTask<SmoothDriving::NewStateType::lastEndState>(SmoothDriving::Accelerate(30, 30.0f));
							break;
						case HeadingDirection::east:
							SmoothDriving::setNewTask<SmoothDriving::NewStateType::lastEndState>(SmoothDriving::Stop());
							while (!SmoothDriving::isTaskFinished());
							SmoothDriving::setNewTask<SmoothDriving::NewStateType::lastEndState>(SmoothDriving::Rotate(5, 180.0f));
							while (!SmoothDriving::isTaskFinished());
							SmoothDriving::setNewTask<SmoothDriving::NewStateType::lastEndState>(SmoothDriving::Accelerate(30, 30.0f));
							break;
						case HeadingDirection::south:
							SmoothDriving::setNewTask<SmoothDriving::NewStateType::lastEndState>(SmoothDriving::Stop());
							while (!SmoothDriving::isTaskFinished());
							SmoothDriving::setNewTask<SmoothDriving::NewStateType::lastEndState>(SmoothDriving::Rotate(-5, -90.0f));
							while (!SmoothDriving::isTaskFinished());
							SmoothDriving::setNewTask<SmoothDriving::NewStateType::lastEndState>(SmoothDriving::Accelerate(30, 30.0f));
							break;
						case HeadingDirection::west:
							SmoothDriving::setNewTask<SmoothDriving::NewStateType::lastEndState>(SmoothDriving::Stop());
							while (!SmoothDriving::isTaskFinished());
							SmoothDriving::setNewTask<SmoothDriving::NewStateType::lastEndState>(SmoothDriving::Accelerate(30, 30.0f));
							break;
						default:
							break;
						}
					}
				}
			}
		}
	}
}