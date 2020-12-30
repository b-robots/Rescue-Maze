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
			static GridCell cell;
			static RelativeDir relativeTurnDir;
			static bool found = false;
			static bool aligned = false;
			static bool frontIsWall = false;

			const auto tempFusedData = SensorFusion::getFusedData();

			if (tempFusedData.gridCellCertainty >= 0.5f)
			{
				if (SmoothDriving::isTaskFinished())
				{
					cell = tempFusedData.gridCell;

					switch (makeAbsolute(RelativeDir::forward, tempFusedData.robotState.heading))
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
						//SmoothDriving::setNewTask<SmoothDriving::NewStateType::lastEndState>(SmoothDriving::AlignFront());
						aligned = true;
						return;
					}

					found = false;

					while (!found)
					{
						uint8_t rand = random(0, 4);

						if ((cell.cellConnections & Directions::north) && rand == 0)
						{
							found = true;
							relativeTurnDir = makeRelative(AbsoluteDir::north, tempFusedData.robotState.heading);
						}
						else if ((cell.cellConnections & Directions::east) && rand == 1)
						{
							found = true;
							relativeTurnDir = makeRelative(AbsoluteDir::east, tempFusedData.robotState.heading);
						}
						else if ((cell.cellConnections & Directions::west) && rand == 2)
						{
							found = true;
							relativeTurnDir = makeRelative(AbsoluteDir::west, tempFusedData.robotState.heading);
						}
						else if ((cell.cellConnections & Directions::south) && rand == 3)
						{
							found = true;
							relativeTurnDir = makeRelative(AbsoluteDir::south, tempFusedData.robotState.heading);
						}
					}

					if (aligned)
					{
						Vec3f position = tempFusedData.robotState.position;
						Vec3f rotation = tempFusedData.robotState.rotation;

						position.x = roundf(position.x / JAFDSettings::Field::cellWidth) * JAFDSettings::Field::cellWidth;
						position.y = roundf(position.y / JAFDSettings::Field::cellWidth) * JAFDSettings::Field::cellWidth;

						while (rotation.x < 0.0f) rotation.x += M_TWOPI;
						while (rotation.x > M_TWOPI) rotation.x -= M_TWOPI;

						if (rotation.x > 315.0f * DEG_TO_RAD || rotation.x < 45.0f * DEG_TO_RAD)
						{
							rotation.x = 0.0f;
							position.x += JAFDSettings::Field::cellWidth / 2.0f - JAFDSettings::SmoothDriving::minAlignDist / 10.0f - JAFDSettings::Mechanics::distSensFrontBackDist / 2.0f;
						}
						else if (rotation.x > 45.0f * DEG_TO_RAD && rotation.x < 135.0f * DEG_TO_RAD)
						{
							rotation.x = 90.0f * DEG_TO_RAD;
							position.y += JAFDSettings::Field::cellWidth / 2.0f - JAFDSettings::SmoothDriving::minAlignDist / 10.0f - JAFDSettings::Mechanics::distSensFrontBackDist / 2.0f;
						}
						else if (rotation.x > 135.0f * DEG_TO_RAD && rotation.x < 225.0f * DEG_TO_RAD)
						{
							rotation.x = 180.0f * DEG_TO_RAD;
							position.x -= JAFDSettings::Field::cellWidth / 2.0f - JAFDSettings::SmoothDriving::minAlignDist / 10.0f - JAFDSettings::Mechanics::distSensFrontBackDist / 2.0f;
						}
						else
						{
							rotation.x = 270.0f * DEG_TO_RAD;
							position.y -= JAFDSettings::Field::cellWidth / 2.0f - JAFDSettings::SmoothDriving::minAlignDist / 10.0f - JAFDSettings::Mechanics::distSensFrontBackDist / 2.0f;
						}

						//SensorFusion::setCertainRobotPosition(position, rotation);
					}

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

					aligned = false;
				}
			}
		}

		void timeBetweenUpdate()
		{
			//CamRec::loop();
		}
	}
}