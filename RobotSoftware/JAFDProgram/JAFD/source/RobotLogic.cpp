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
			static bool found = false;
			static bool aligned = false;
			static bool frontIsWall = false;

			if (SensorFusion::getFusedData().gridCellCertainty >= 0.5f)
			{
				if (SmoothDriving::isTaskFinished())
				{
					cell = SensorFusion::getFusedData().gridCell;

					switch (makeAbsolute(RelativeDir::forward, SensorFusion::getFusedData().heading))
					{
					case AbsoluteDir::north:
						if (cell.cellConnections & Directions::north) frontIsWall = false;
						else frontIsWall = true;
						break;
					case AbsoluteDir::east:
						if (cell.cellConnections & Directions::east) frontIsWall = false;
						else frontIsWall = true;
						break;
					case AbsoluteDir::south:
						if (cell.cellConnections & Directions::south) frontIsWall = false;
						else frontIsWall = true;
						break;
					case AbsoluteDir::west:
						if (cell.cellConnections & Directions::west) frontIsWall = false;
						else frontIsWall = true;
						break;
					default:
						break;
					}

					if (frontIsWall && !aligned)
					{
						SmoothDriving::setNewTask<SmoothDriving::NewStateType::lastEndState>(SmoothDriving::AlignFront(30));
						aligned = true;
						return;
					}

					found = false;

					while (!found)
					{
						if ((cell.cellConnections & Directions::north) && random(0, 4) == 0)
						{
							found = true;
							relativeTurnDir = makeRelative(AbsoluteDir::north, SensorFusion::getFusedData().heading);
						}
						else if ((cell.cellConnections & Directions::east) && random(0, 4) == 1)
						{
							found = true;
							relativeTurnDir = makeRelative(AbsoluteDir::east, SensorFusion::getFusedData().heading);
						}
						else if ((cell.cellConnections & Directions::west) && random(0, 4) == 2)
						{
							found = true;
							relativeTurnDir = makeRelative(AbsoluteDir::west, SensorFusion::getFusedData().heading);
						}
						else if ((cell.cellConnections & Directions::south) && random(0, 4) == 3)
						{
							found = true;
							relativeTurnDir = makeRelative(AbsoluteDir::south, SensorFusion::getFusedData().heading);
						}
					}

					if (aligned)
					{
						Vec3f position = SensorFusion::getFusedData().robotState.position;
						Vec3f rotation = SensorFusion::getFusedData().robotState.rotation;

						position.x = roundf(position.x / JAFDSettings::Field::cellWidth) * JAFDSettings::Field::cellWidth;
						position.y = roundf(position.y / JAFDSettings::Field::cellWidth) * JAFDSettings::Field::cellWidth;

						while (rotation.x < 0.0f) rotation.x += M_TWOPI;
						while (rotation.x > M_TWOPI) rotation.x -= M_TWOPI;

						if (rotation.x > 315.0f * DEG_TO_RAD || rotation.x < 45.0f * DEG_TO_RAD)
						{
							rotation.x = 0.0f;
							position.x += JAFDSettings::Field::cellWidth / 2.0f - 3 - JAFDSettings::Mechanics::sensorFrontBackDist / 2.0f;
						}
						else if (rotation.x > 45.0f * DEG_TO_RAD && rotation.x < 135.0f * DEG_TO_RAD)
						{
							rotation.x = 90.0f * DEG_TO_RAD;
							position.y += JAFDSettings::Field::cellWidth / 2.0f - 3 - JAFDSettings::Mechanics::sensorFrontBackDist / 2.0f;

						}
						else if (rotation.x > 135.0f * DEG_TO_RAD && rotation.x < 225.0f * DEG_TO_RAD)
						{
							rotation.x = 180.0f * DEG_TO_RAD;
							position.x -= JAFDSettings::Field::cellWidth / 2.0f - 3 - JAFDSettings::Mechanics::sensorFrontBackDist / 2.0f;

						}
						else
						{
							rotation.x = 270.0f * DEG_TO_RAD;
							position.y -= JAFDSettings::Field::cellWidth / 2.0f - 3 - JAFDSettings::Mechanics::sensorFrontBackDist / 2.0f;

						}

						SensorFusion::setCertainRobotPosition(position, rotation);
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

					aligned = false;
				}
			}
		}
	}
}