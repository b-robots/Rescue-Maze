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
			static RelativeDir relativeTurnDir;
			
			if (SensorFusion::getFusedData().gridCellCertainty >= 0.5f)
			{
				if (SmoothDriving::isTaskFinished())
				{
					cell = SensorFusion::getFusedData().gridCell;

					if (cell.cellConnections & Direction::north)
					{
						relativeTurnDir = makeRelative(HeadingDirection::north, SensorFusion::getFusedData().heading);
					}
					else if (cell.cellConnections & Direction::east)
					{
						relativeTurnDir = makeRelative(HeadingDirection::east, SensorFusion::getFusedData().heading);
					}
					else if (cell.cellConnections & Direction::south)
					{
						relativeTurnDir = makeRelative(HeadingDirection::south, SensorFusion::getFusedData().heading);
					}
					else if (cell.cellConnections & Direction::west)
					{
						relativeTurnDir = makeRelative(HeadingDirection::west, SensorFusion::getFusedData().heading);
					}

					switch (relativeTurnDir)
					{
					case RelativeDir::forward:
						SmoothDriving::setNewTask<SmoothDriving::NewStateType::lastEndState>(SmoothDriving::TaskArray(	SmoothDriving::Stop(),
																														SmoothDriving::Accelerate(20, 15.0f),
																														SmoothDriving::Accelerate(0, 15.0f),
																														SmoothDriving::Stop()));
						break;
					case RelativeDir::right:
						SmoothDriving::setNewTask<SmoothDriving::NewStateType::lastEndState>(SmoothDriving::TaskArray(	SmoothDriving::Stop(),
																														SmoothDriving::Rotate(-2.0f, -90.0f),
																														SmoothDriving::Accelerate(20, 15.0f),
																														SmoothDriving::Accelerate(0, 15.0f),
																														SmoothDriving::Stop()));
						break;
					case RelativeDir::backward:
						SmoothDriving::setNewTask<SmoothDriving::NewStateType::lastEndState>(SmoothDriving::TaskArray(	SmoothDriving::Stop(),
																														SmoothDriving::Rotate(3.0f, 180.0f),
																														SmoothDriving::Accelerate(20, 15.0f),
																														SmoothDriving::Accelerate(0, 15.0f),
																														SmoothDriving::Stop()));
						break;
					case RelativeDir::left:
						SmoothDriving::setNewTask<SmoothDriving::NewStateType::lastEndState>(SmoothDriving::TaskArray(	SmoothDriving::Stop(),
																														SmoothDriving::Rotate(2.0f, 90.0f),
																														SmoothDriving::Accelerate(20, 15.0f),
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