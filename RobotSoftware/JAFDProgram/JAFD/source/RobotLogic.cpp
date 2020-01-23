/*
This is the heart of the robot
*/

#include "../header/RobotLogic.h"
#include "../header/SensorFusion.h"
#include "../header/MazeMapping.h"

namespace JAFD
{
	namespace RobotLogic
	{
		void loop()
		{
			if (SensorFusion::getFusedData().gridCellCertainty >= 0.5f)
			{
				GridCell cell = SensorFusion::getFusedData().gridCell;

				if (SmoothDriving::isTaskFinished())
				{
					if (cell.cellConnections & Direction::north)
					{
						switch (SensorFusion::getFusedData().heading)
						{
						case HeadingDirection::north:
							SmoothDriving::setNewTask<SmoothDriving::NewStateType::currentState>(SmoothDriving::Accelerate(30, 30.0f), true);
							break;
						case HeadingDirection::east:
							tempCell.cellConnections |= Direction::north;
							break;
						case HeadingDirection::south:
							tempCell.cellConnections |= Direction::east;
							break;
						case HeadingDirection::west:
							tempCell.cellConnections |= Direction::south;
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