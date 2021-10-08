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
		void loop()
		{
			GridCell cell;
			RelativeDir relativeTurnDir;
			bool found = false;

			const auto tempFusedData = SensorFusion::getFusedData();

			if (tempFusedData.gridCellCertainty >= 0.5f)
			{
				switch (relativeTurnDir)
				{
				case RelativeDir::forward:
					SmoothDriving::setNewTask<SmoothDriving::NewStateType::lastEndState>(SmoothDriving::TaskArray(SmoothDriving::Stop(),
						SmoothDriving::Accelerate(20, 15.0f),
						SmoothDriving::Accelerate(0, 15.0f),
						SmoothDriving::Stop()));
					break;
				case RelativeDir::right:
					SmoothDriving::setNewTask<SmoothDriving::NewStateType::lastEndState>(SmoothDriving::TaskArray(SmoothDriving::Stop(),
						SmoothDriving::Rotate(-2.0f, -90.0f),
						SmoothDriving::Accelerate(20, 15.0f),
						SmoothDriving::Accelerate(0, 15.0f),
						SmoothDriving::Stop()));
					break;
				case RelativeDir::backward:
					SmoothDriving::setNewTask<SmoothDriving::NewStateType::lastEndState>(SmoothDriving::TaskArray(SmoothDriving::Stop(),
						SmoothDriving::Rotate(3.0f, 180.0f),
						SmoothDriving::Accelerate(20, 15.0f),
						SmoothDriving::Accelerate(0, 15.0f),
						SmoothDriving::Stop()));
					break;
				case RelativeDir::left:
					SmoothDriving::setNewTask<SmoothDriving::NewStateType::lastEndState>(SmoothDriving::TaskArray(SmoothDriving::Stop(),
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

		void timeBetweenUpdate()
		{
			// DEBUG
			//CamRec::loop();
		}
	}
}