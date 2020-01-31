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
			static GridCell cell;
			static HeadingDirection relativeTurnDir;
			
			if (SensorFusion::getFusedData().gridCellCertainty >= 0.5f)
			{
				if (SmoothDriving::isTaskFinished())
				{
					cell = SensorFusion::getFusedData().gridCell;

					if (cell.cellConnections & Direction::north)
					{
						switch (SensorFusion::getFusedData().heading)
						{
						case HeadingDirection::north:
							relativeTurnDir = HeadingDirection::north;
							break;
						case HeadingDirection::east:
							relativeTurnDir = HeadingDirection::east;
							break;
						case HeadingDirection::south:
							relativeTurnDir = HeadingDirection::south;
							break;
						case HeadingDirection::west:
							relativeTurnDir = HeadingDirection::west;
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
							relativeTurnDir = HeadingDirection::east;
							break;
						case HeadingDirection::east:
							relativeTurnDir = HeadingDirection::north;
							break;
						case HeadingDirection::south:
							relativeTurnDir = HeadingDirection::west;
							break;
						case HeadingDirection::west:
							relativeTurnDir = HeadingDirection::south;
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
							relativeTurnDir = HeadingDirection::south;
							break;
						case HeadingDirection::east:
							relativeTurnDir = HeadingDirection::east;
							break;
						case HeadingDirection::south:
							relativeTurnDir = HeadingDirection::north;
							break;
						case HeadingDirection::west:
							relativeTurnDir = HeadingDirection::west;
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
							relativeTurnDir = HeadingDirection::west;
							break;
						case HeadingDirection::east:
							relativeTurnDir = HeadingDirection::south;
							break;
						case HeadingDirection::south:
							relativeTurnDir = HeadingDirection::east;
							break;
						case HeadingDirection::west:
							relativeTurnDir = HeadingDirection::north;
							break;
						default:
							break;
						}
					}

					switch (relativeTurnDir)
					{
					case HeadingDirection::north:
						SmoothDriving::setNewTask<SmoothDriving::NewStateType::lastEndState>(SmoothDriving::TaskArray(	SmoothDriving::Stop(),
																														SmoothDriving::Accelerate(30, 15.0f),
																														SmoothDriving::Accelerate(0, 15.0f),
																														SmoothDriving::Stop()));
						break;
					case HeadingDirection::east:
						SmoothDriving::setNewTask<SmoothDriving::NewStateType::lastEndState>(SmoothDriving::TaskArray(	SmoothDriving::Stop(),
																														SmoothDriving::Rotate(-3, -270.0f),
																														SmoothDriving::Accelerate(30, 15.0f),
																														SmoothDriving::Accelerate(0, 15.0f),
																														SmoothDriving::Stop()));
						break;
					case HeadingDirection::south:
						SmoothDriving::setNewTask<SmoothDriving::NewStateType::lastEndState>(SmoothDriving::TaskArray(	SmoothDriving::Stop(),
																														SmoothDriving::Rotate(3, 180.0f),
																														SmoothDriving::Accelerate(30, 15.0f),
																														SmoothDriving::Accelerate(0, 15.0f),
																														SmoothDriving::Stop()));
						break;
					case HeadingDirection::west:
						SmoothDriving::setNewTask<SmoothDriving::NewStateType::lastEndState>(SmoothDriving::TaskArray(	SmoothDriving::Stop(),
																														SmoothDriving::Rotate(3, 270.0f),
																														SmoothDriving::Accelerate(30, 15.0f),
																														SmoothDriving::Accelerate(0, 15.0f),
																														SmoothDriving::Stop()));
						break;
					default:
						break;
					}
				}
			}
		}
	}
}