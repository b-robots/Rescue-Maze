/*
This is the heart of the robot
*/

#include "../header/RobotLogic.h"
#include "../header/SensorFusion.h"
#include "../header/MazeMapping.h"
#include "../header/SmoothDriving.h"
#include "../header/MotorControl.h"
#include "../header/HeatSensor.h"
#include "../header/Dispenser.h"
#include "../header/CamRec.h"
#include "../header/TCS34725.h"

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
					setNewTask<NewStateType::lastEndState>(TaskArray(Stop(),
						FollowWall(25, 30.0f),
						AlignWalls(),
						Stop()));
					break;
				case RelativeDir::right:
					setNewTask<NewStateType::lastEndState>(TaskArray(Stop(),
						Rotate(-1.0f, -90.0f),
						AlignWalls(),
						FollowWall(25, 30.0f),
						AlignWalls(),
						Stop()));
					break;
				case RelativeDir::backward:
					setNewTask<NewStateType::lastEndState>(TaskArray(Stop(),
						Rotate(1.0f, 180.0f),
						AlignWalls(),
						FollowWall(25, 30.0f),
						AlignWalls(),
						Stop()));
					break;
				case RelativeDir::left:
					setNewTask<NewStateType::lastEndState>(TaskArray(Stop(),
						Rotate(1.0f, 90.0f),
						AlignWalls(),
						FollowWall(25, 30.0f),
						AlignWalls(),
						Stop()));
					break;
				default:
					break;
				}
			}
		}

		// Start task which prefers straigth over left over right over back
		// Intended for the first maneuver after starting
		void startFirstTask(FusedData tempFusedData) {
			AbsoluteDir absLeft = makeAbsolute(RelativeDir::left, tempFusedData.robotState.heading);
			AbsoluteDir absForward = makeAbsolute(RelativeDir::forward, tempFusedData.robotState.heading);
			AbsoluteDir absRight = makeAbsolute(RelativeDir::right, tempFusedData.robotState.heading);
			AbsoluteDir absBack = makeAbsolute(RelativeDir::backward, tempFusedData.robotState.heading);

			if (tempFusedData.gridCell.cellConnections & static_cast<uint8_t>(absForward)) {
				startRelativeTurnDirDrive(RelativeDir::forward);
			}
			else
			{
				if (tempFusedData.gridCell.cellConnections & static_cast<uint8_t>(absLeft) && tempFusedData.gridCell.cellConnections & static_cast<uint8_t>(absRight))
				{
					if (random(2) == 0) {
						Serial.println("start first left");
						startRelativeTurnDirDrive(RelativeDir::left);
					}
					else {
						Serial.println("start first right");
						startRelativeTurnDirDrive(RelativeDir::right);
					}
				}
				else if (tempFusedData.gridCell.cellConnections & static_cast<uint8_t>(absLeft)) {
					Serial.println("start first left");
					startRelativeTurnDirDrive(RelativeDir::left);
				}
				else if (tempFusedData.gridCell.cellConnections & static_cast<uint8_t>(absRight)) {
					Serial.println("start first right");
					startRelativeTurnDirDrive(RelativeDir::right);
				}
				else if (tempFusedData.gridCell.cellConnections & static_cast<uint8_t>(absBack)) {
					Serial.println("start first back");
					startRelativeTurnDirDrive(RelativeDir::backward);
				}
				else {
					Serial.println("No direction to drive!");
					return;
				}
			}
		}

		bool getNextDrivingDir(FusedData tempFusedData, RelativeDir& nextDir) {
			const AbsoluteDir absLeft = makeAbsolute(RelativeDir::left, tempFusedData.robotState.heading);
			const AbsoluteDir absForward = makeAbsolute(RelativeDir::forward, tempFusedData.robotState.heading);
			const AbsoluteDir absRight = makeAbsolute(RelativeDir::right, tempFusedData.robotState.heading);
			const AbsoluteDir absBack = makeAbsolute(RelativeDir::backward, tempFusedData.robotState.heading);

			struct {
				bool F = true;
				bool R = true;
				bool L = true;
				bool B = true;
			} possibleRelDir;

			if (!(tempFusedData.gridCell.cellConnections & (uint8_t) absLeft)) {
				possibleRelDir.L = false;
			}

			if (!(tempFusedData.gridCell.cellConnections & (uint8_t)absRight)) {
				possibleRelDir.R = false;
			}

			if (!(tempFusedData.gridCell.cellConnections & (uint8_t)absForward)) {
				possibleRelDir.F = false;
			}

			if (!(tempFusedData.gridCell.cellConnections & (uint8_t)absBack)) {
				possibleRelDir.B = false;
			}

			GridCell cell;

			if (possibleRelDir.L) {
				MazeMapping::getGridCell(&cell, tempFusedData.robotState.mapCoordinate.getCoordinateInDir(absLeft));
				if (cell.cellState & CellState::blackTile) {
					possibleRelDir.L = false;
				}
			}

			if (possibleRelDir.R) {
				MazeMapping::getGridCell(&cell, tempFusedData.robotState.mapCoordinate.getCoordinateInDir(absRight));
				if (cell.cellState & CellState::blackTile) {
					possibleRelDir.R = false;
				}
			}

			if (possibleRelDir.F) {
				MazeMapping::getGridCell(&cell, tempFusedData.robotState.mapCoordinate.getCoordinateInDir(absForward));
				if (cell.cellState & CellState::blackTile) {
					possibleRelDir.F = false;
				}
			}

			if (possibleRelDir.B) {
				MazeMapping::getGridCell(&cell, tempFusedData.robotState.mapCoordinate.getCoordinateInDir(absBack));
				if (cell.cellState & CellState::blackTile) {
					possibleRelDir.B = false;
				}
			}

			if (possibleRelDir.F) {
				nextDir = RelativeDir::forward;
			}
			else if (possibleRelDir.L && possibleRelDir.R) {
				if (random(2) == 0) {
					nextDir = RelativeDir::left;
				}
				else {
					nextDir = RelativeDir::right;
				}
			}
			else if (possibleRelDir.L) {
				nextDir = RelativeDir::left;
			}
			else if (possibleRelDir.R) {
				nextDir = RelativeDir::right;
			}
			else if (possibleRelDir.B) {
				nextDir = RelativeDir::backward;
			}
			else {
				nextDir = RelativeDir::forward;
				Serial.println("No direction to drive! (2)");
				return false;
			}

			return true;

			// TODO
		}

		void loop()
		{
			using namespace SmoothDriving;

			static bool start = true;
			static bool startAfterRotation = false;
			static MapCoordinate nextMapCoordinate;
			static AbsoluteDir nextHeading;

			const FusedData tempFusedData = SensorFusion::getFusedData();

			if (start) {
				if (!SensorFusion::scanSurrounding()) {
					return;
				}

				SensorFusion::updatePosAndRotFromDist();

				if (tempFusedData.gridCell.cellConnections & EntranceDirections::east) {
					Serial.println("start rot. east");
					setNewTask<NewStateType::lastEndState>(TaskArray(Stop(),
						Rotate(-1.0f, -90.0f),
						AlignWalls(),
						Stop()));
					startAfterRotation = true;
				}
				else {
					Serial.println("start rot. west");
					setNewTask<NewStateType::lastEndState>(TaskArray(Stop(),
						Rotate(1.0f, 90.0f),
						AlignWalls(),
						Stop()));
					startAfterRotation = true;
				}

				start = false;
			}
			else if (startAfterRotation) {
				if (!SmoothDriving::isTaskFinished()) {
					return;
				}

				if (!SensorFusion::scanSurrounding()) {
					return;
				}

				Serial.println("startAfterRotation -> updatePos");
				SensorFusion::updatePosAndRotFromDist();

				startFirstTask(tempFusedData);

				startAfterRotation = false;
			}
			else {
				static int consecutiveLHeat = 0;
				static int consecutiveRHeat = 0;
				static bool leftHeat = false;
				static bool rightHeat = false;
				static MapCoordinate lastHeatVictim = homePosition;

				if (HeatSensor::detectVictim(HeatSensorSide::left) &&
					!(tempFusedData.gridCell.cellConnections & static_cast<uint8_t>(makeAbsolute(RelativeDir::left, tempFusedData.robotState.heading)))) {
					consecutiveLHeat++;
				}
				else {
					consecutiveLHeat = 0;
				}

				if (HeatSensor::detectVictim(HeatSensorSide::right) &&
					!(tempFusedData.gridCell.cellConnections & static_cast<uint8_t>(makeAbsolute(RelativeDir::right, tempFusedData.robotState.heading)))) {
					consecutiveRHeat++;
				}
				else {
					consecutiveRHeat = 0;
				}

				if (consecutiveLHeat > 5 && lastHeatVictim != tempFusedData.robotState.mapCoordinate)
					leftHeat = true;
				else if (consecutiveRHeat > 5 && lastHeatVictim != tempFusedData.robotState.mapCoordinate)
					rightHeat = true;

				if (rightHeat || leftHeat) {
					Serial.println("heat");
					lastHeatVictim = tempFusedData.robotState.mapCoordinate;

					GridCell currentCell;
					MazeMapping::getGridCell(&currentCell, tempFusedData.robotState.mapCoordinate);

					if (!(currentCell.cellState & CellState::victim)) {
						if (isTaskFinished() || isDrivingStraight()) {
							Serial.println("start disp");
							auto startDisp = millis();

							SmoothDriving::stopTask();

							currentCell.cellState |= CellState::victim;
							MazeMapping::setGridCell(currentCell, tempFusedData.robotState.mapCoordinate);

							if (leftHeat) {
								Dispenser::dispenseLeft(1);
							}
							else {
								Dispenser::dispenseRight(1);
							}

							delay(6000 - (millis() - startDisp));
						}
					}
				}

				static bool returnFromBlack = false;

				if (isDrivingStraight()) {
					FloorTileColour tileColour = ColorSensor::detectTileColour(tempFusedData.colorSensData.lux);

					if (tileColour == FloorTileColour::black && !returnFromBlack) {
						Serial.println("start return from black");
						Serial.println(tempFusedData.robotState.position.y);
						setNewTask<NewStateType::lastEndState>(TaskArray(FollowWall(-25, -30.0f), AlignWalls(), Stop()), true);
						returnFromBlack = true;
						return;
					}
				}

				if (SmoothDriving::isTaskFinished()) {
					if (returnFromBlack) {
						GridCell cell;
						MazeMapping::getGridCell(&cell, tempFusedData.robotState.mapCoordinate.getCoordinateInDir(tempFusedData.robotState.heading));
						cell.cellState = cell.cellState | CellState::blackTile;
						MazeMapping::setGridCell(cell, tempFusedData.robotState.mapCoordinate.getCoordinateInDir(tempFusedData.robotState.heading));
						Serial.println(tempFusedData.robotState.position.y);
						Serial.println("store black tile");
						returnFromBlack = false;
					}

					if (SensorFusion::scanSurrounding()) {
						Serial.println("scanned surr. -> update pos");
						SensorFusion::updatePosAndRotFromDist();

						RelativeDir driveDir;
						if (getNextDrivingDir(tempFusedData, driveDir)) {
							Serial.print("start new turn dir. drive: ");
							Serial.println((int)driveDir);
							startRelativeTurnDirDrive(driveDir);
						}
					}
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