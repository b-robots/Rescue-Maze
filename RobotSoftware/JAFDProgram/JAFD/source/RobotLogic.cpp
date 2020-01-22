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
			auto gridCellCertainty = MazeMapping::updateCurrentCell();

			if (gridCellCertainty >= 0.5f)
			{
				GridCell cell = SensorFusion::getFusedData().gridCell;
			}
		}
	}
}