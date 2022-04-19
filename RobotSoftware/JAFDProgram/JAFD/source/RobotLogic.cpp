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

			// Check for walls
			if (!(tempFusedData.gridCell.cellConnections & (uint8_t)absLeft)) {
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

			// Check for black tile
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

			// Check for already visited tiles
			decltype(possibleRelDir) possibleUnvisitedRelDir = possibleRelDir;

			if (possibleUnvisitedRelDir.L) {
				MazeMapping::getGridCell(&cell, tempFusedData.robotState.mapCoordinate.getCoordinateInDir(absLeft));
				if (cell.cellState & CellState::visited) {
					possibleUnvisitedRelDir.L = false;
				}
			}

			if (possibleUnvisitedRelDir.R) {
				MazeMapping::getGridCell(&cell, tempFusedData.robotState.mapCoordinate.getCoordinateInDir(absRight));
				if (cell.cellState & CellState::visited) {
					possibleUnvisitedRelDir.R = false;
				}
			}

			if (possibleUnvisitedRelDir.F) {
				MazeMapping::getGridCell(&cell, tempFusedData.robotState.mapCoordinate.getCoordinateInDir(absForward));
				if (cell.cellState & CellState::visited) {
					possibleUnvisitedRelDir.F = false;
				}
			}

			if (possibleUnvisitedRelDir.B) {
				MazeMapping::getGridCell(&cell, tempFusedData.robotState.mapCoordinate.getCoordinateInDir(absBack));
				if (cell.cellState & CellState::visited) {
					possibleUnvisitedRelDir.B = false;
				}
			}

			// Select possible unvisited dir (prefer forward, left/right are chosen randomly)
			if (possibleUnvisitedRelDir.F) {
				nextDir = RelativeDir::forward;
			}
			else if (possibleUnvisitedRelDir.L && possibleUnvisitedRelDir.R) {
				if (random(2) == 0) {
					nextDir = RelativeDir::left;
				}
				else {
					nextDir = RelativeDir::right;
				}
			}
			else if (possibleUnvisitedRelDir.L) {
				nextDir = RelativeDir::left;
			}
			else if (possibleUnvisitedRelDir.R) {
				nextDir = RelativeDir::right;
			}
			else if (possibleUnvisitedRelDir.B) {
				nextDir = RelativeDir::backward;
			}
			else {
				nextDir = RelativeDir::forward;
				Serial.println("No univisted direction to drive!");
			}

			// Select possible dir (prefer forward, left/right are chosen randomly)
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
				Serial.println("No direction to drive!");
				return false;
			}

			return true;

			// TODO
		}

		void handleVictimDetection(FusedData tempFusedData, uint8_t sureWalls) {
			// TODO

			// heat victims
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

			if (consecutiveLHeat > 3 && lastHeatVictim != tempFusedData.robotState.mapCoordinate)
				leftHeat = true;
			else if (consecutiveRHeat > 3 && lastHeatVictim != tempFusedData.robotState.mapCoordinate)
				rightHeat = true;

			if (rightHeat || leftHeat) {
				Serial.println("heat");
				lastHeatVictim = tempFusedData.robotState.mapCoordinate;

				GridCell currentCell;
				MazeMapping::getGridCell(&currentCell, tempFusedData.robotState.mapCoordinate);

				if (!(currentCell.cellState & CellState::victim)) {
					if (SmoothDriving::isTaskFinished() || SmoothDriving::isDrivingStraight()) {
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

						leftHeat = false;
						rightHeat = false;
					}
				}
			}
		}

		void loop()
		{
			using namespace SmoothDriving;

			static MapCoordinate lastCoordinate = homePosition;

			static bool start = true;
			static bool startAfterRotation = false;
			static uint8_t cumSureWalls = 0b0000;
			static bool blockCellChange = false;

			const FusedData tempFusedData = SensorFusion::getFusedData();

			if (isTaskFinished) {
				blockCellChange = false;
			}

			if (start) {
				if (!SensorFusion::scanSurrounding(cumSureWalls)) {
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

				if (!SensorFusion::scanSurrounding(cumSureWalls)) {
					return;
				}

				Serial.println("startAfterRotation -> updatePos");
				SensorFusion::updatePosAndRotFromDist();

				startFirstTask(tempFusedData);

				startAfterRotation = false;
			}
			else {
				handleVictimDetection(tempFusedData, cumSureWalls);

				static bool returnFromBlack = false;

				if (isDrivingStraight()) {
					FloorTileColour tileColour = ColorSensor::detectTileColour(tempFusedData.colorSensData.lux);

					if (tileColour == FloorTileColour::black && !returnFromBlack) {
						Serial.println("start return from black");
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
						Serial.println("store black tile");
						returnFromBlack = false;
					}

					if (SensorFusion::scanSurrounding(cumSureWalls)) {
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

			if (tempFusedData.robotState.mapCoordinate != lastCoordinate && !blockCellChange) {
				Serial.print("left cell - x: ");
				Serial.print(lastCoordinate.x);
				Serial.print(", y: ");
				Serial.print(lastCoordinate.y);
				Serial.print(", walls: ");
				Serial.println(cumSureWalls, BIN);

				GridCell currCell;
				MazeMapping::getGridCell(&currCell, lastCoordinate);
				currCell.cellConnections = (currCell.cellConnections & ~CellConnections::directionMask) | ~cumSureWalls;
				currCell.cellState |= CellState::visited;
				MazeMapping::setGridCell(currCell, lastCoordinate);
				cumSureWalls = 0b0000;

				blockCellChange = true;
			}

			lastCoordinate = tempFusedData.robotState.mapCoordinate;
		}

		void timeBetweenUpdate()
		{
			CamRec::loop();
		}
	}
}