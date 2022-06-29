#include "../header/RobotLogic.h"
#include "../header/SensorFusion.h"
#include "../header/MazeMapping.h"
#include "../header/SmoothDriving.h"
#include "../header/MotorControl.h"
#include "../header/HeatSensor.h"
//#include "../header/Dispenser.h"
#include "../header/ColorSensor.h"
#include "../header/Math.h"
#include "../header/SmallThings.h"

namespace SIAL
{
	namespace RobotLogic
	{
		void startRelativeTurnDirDrive(RelativeDir dir, FusedData fusedData) {
			using namespace SmoothDriving;

			switch (dir)
			{
			case RelativeDir::forward:
				startNewTask(new TaskArray({
					new FollowCell(30),
					new AlignWalls() }));
				break;
			case RelativeDir::right:
				startNewTask(new TaskArray({
					new Rotate(-M_PI_2, -2.0f),
					new AlignWalls(),
					new FollowCell(30),
					new AlignWalls() }));
				break;
			case RelativeDir::backward:
				startNewTask(new TaskArray({
					new Rotate(M_PI, 2.0f),
					new AlignWalls(),
					new FollowCell(30),
					new AlignWalls() }));
				break;
			case RelativeDir::left:
				startNewTask(new TaskArray({
					new Rotate(M_PI_2, 2.0f),
					new AlignWalls(),
					new FollowCell(30),
					new AlignWalls() }));
				break;
			default:
				break;
			}
		}

		void startFirstTask(FusedData fusedData) {
			AbsoluteDir absLeft = makeAbsolute(RelativeDir::left, fusedData.robotState.heading);
			AbsoluteDir absForward = makeAbsolute(RelativeDir::forward, fusedData.robotState.heading);
			AbsoluteDir absRight = makeAbsolute(RelativeDir::right, fusedData.robotState.heading);
			AbsoluteDir absBack = makeAbsolute(RelativeDir::backward, fusedData.robotState.heading);

			if (fusedData.gridCell.cellConnections & static_cast<uint8_t>(absForward)) {
				startRelativeTurnDirDrive(RelativeDir::forward, fusedData);
			}
			else
			{
				if (fusedData.gridCell.cellConnections & static_cast<uint8_t>(absLeft) && fusedData.gridCell.cellConnections & static_cast<uint8_t>(absRight))
				{
					if (random(2) == 0) {
						Serial.println("start first left");
						startRelativeTurnDirDrive(RelativeDir::left, fusedData);
					}
					else {
						Serial.println("start first right");
						startRelativeTurnDirDrive(RelativeDir::right, fusedData);
					}
				}
				else if (fusedData.gridCell.cellConnections & static_cast<uint8_t>(absLeft)) {
					Serial.println("start first left");
					startRelativeTurnDirDrive(RelativeDir::left, fusedData);
				}
				else if (fusedData.gridCell.cellConnections & static_cast<uint8_t>(absRight)) {
					Serial.println("start first right");
					startRelativeTurnDirDrive(RelativeDir::right, fusedData);
				}
				else if (fusedData.gridCell.cellConnections & static_cast<uint8_t>(absBack)) {
					Serial.println("start first back");
					startRelativeTurnDirDrive(RelativeDir::backward, fusedData);
				}
				else {
					Serial.println("No direction to drive!");
					return;
				}
			}
		}

		bool getNextDrivingDir(FusedData fusedData, RelativeDir& nextDir) {
			const AbsoluteDir absLeft = makeAbsolute(RelativeDir::left, fusedData.robotState.heading);
			const AbsoluteDir absForward = makeAbsolute(RelativeDir::forward, fusedData.robotState.heading);
			const AbsoluteDir absRight = makeAbsolute(RelativeDir::right, fusedData.robotState.heading);
			const AbsoluteDir absBack = makeAbsolute(RelativeDir::backward, fusedData.robotState.heading);

			struct {
				bool F = true;
				bool R = true;
				bool L = true;
				bool B = true;
			} possibleRelDir;

			// Check for walls
			if (!(fusedData.gridCell.cellConnections & (uint8_t)absLeft)) {
				possibleRelDir.L = false;
			}

			if (!(fusedData.gridCell.cellConnections & (uint8_t)absRight)) {
				possibleRelDir.R = false;
			}

			if (!(fusedData.gridCell.cellConnections & (uint8_t)absForward)) {
				possibleRelDir.F = false;
			}

			if (!(fusedData.gridCell.cellConnections & (uint8_t)absBack)) {
				possibleRelDir.B = false;
			}

			GridCell cell;
			decltype(possibleRelDir) possibleOnlyWallsRelDir = possibleRelDir;

			// Check for black
			if (possibleRelDir.L) {
				MazeMapping::getGridCell(&cell, fusedData.robotState.mapCoordinate.getCoordinateInDir(absLeft));
				if (cell.cellState & CellState::blackTile) {
					possibleRelDir.L = false;
				}
			}

			if (possibleRelDir.R) {
				MazeMapping::getGridCell(&cell, fusedData.robotState.mapCoordinate.getCoordinateInDir(absRight));
				if (cell.cellState & CellState::blackTile) {
					possibleRelDir.R = false;
				}
			}

			if (possibleRelDir.F) {
				MazeMapping::getGridCell(&cell, fusedData.robotState.mapCoordinate.getCoordinateInDir(absForward));
				if (cell.cellState & CellState::blackTile) {
					possibleRelDir.F = false;
				}
			}

			if (possibleRelDir.B) {
				MazeMapping::getGridCell(&cell, fusedData.robotState.mapCoordinate.getCoordinateInDir(absBack));
				if (cell.cellState & CellState::blackTile) {
					possibleRelDir.B = false;
				}
			}

			if (possibleRelDir.L) {
				nextDir = RelativeDir::left;
				return true;
			}
			else if (possibleRelDir.F) {
				nextDir = RelativeDir::forward;
				return true;
			}
			else if (possibleRelDir.R) {
				nextDir = RelativeDir::right;
				return true;
			}
			else if (possibleRelDir.B) {
				nextDir = RelativeDir::backward;
				return true;
			}
			else {
				if (possibleOnlyWallsRelDir.L) {
					nextDir = RelativeDir::left;
					return true;
				}
				else if (possibleOnlyWallsRelDir.F) {
					nextDir = RelativeDir::forward;
					return true;
				}
				else if (possibleOnlyWallsRelDir.R) {
					nextDir = RelativeDir::right;
					return true;
				}
				else {
					nextDir = RelativeDir::backward;
					return true;
				}
			}

			// Check for already visited tiles
			decltype(possibleRelDir) possibleUnvisitedRelDir = possibleRelDir;

			if (possibleUnvisitedRelDir.L) {
				MazeMapping::getGridCell(&cell, fusedData.robotState.mapCoordinate.getCoordinateInDir(absLeft));
				if (cell.cellState & CellState::visited) {
					possibleUnvisitedRelDir.L = false;
				}
			}

			if (possibleUnvisitedRelDir.R) {
				MazeMapping::getGridCell(&cell, fusedData.robotState.mapCoordinate.getCoordinateInDir(absRight));
				if (cell.cellState & CellState::visited) {
					possibleUnvisitedRelDir.R = false;
				}
			}

			if (possibleUnvisitedRelDir.F) {
				MazeMapping::getGridCell(&cell, fusedData.robotState.mapCoordinate.getCoordinateInDir(absForward));
				if (cell.cellState & CellState::visited) {
					possibleUnvisitedRelDir.F = false;
				}
			}

			if (possibleUnvisitedRelDir.B) {
				MazeMapping::getGridCell(&cell, fusedData.robotState.mapCoordinate.getCoordinateInDir(absBack));
				if (cell.cellState & CellState::visited) {
					possibleUnvisitedRelDir.B = false;
				}
			}

			// Select possible unvisited dir (prefer forward, left/right are chosen randomly)
			if (possibleUnvisitedRelDir.F) {
				nextDir = RelativeDir::forward;
				return true;
			}
			else if (possibleUnvisitedRelDir.L && possibleUnvisitedRelDir.R) {
				if (random(2) == 0) {
					nextDir = RelativeDir::left;
					return true;
				}
				else {
					return true;
					nextDir = RelativeDir::right;
				}
			}
			else if (possibleUnvisitedRelDir.L) {
				nextDir = RelativeDir::left;
				return true;
			}
			else if (possibleUnvisitedRelDir.R) {
				nextDir = RelativeDir::right;
				return true;
			}
			else if (possibleUnvisitedRelDir.B) {
				nextDir = RelativeDir::backward;
				return true;
			}
			else {
				nextDir = RelativeDir::backward;
				Serial.println("No univisted direction to drive!");
			}
		}
		
		// handleVictimDetection
		/*
				void handleVictimDetection(FusedData tempFusedData) {
					// heat victims
					static int consecutiveLHeat = 0;
					static int consecutiveRHeat = 0;
					static bool leftHeat = false;
					static bool rightHeat = false;
					static MapCoordinate lastHeatVictim = homePosition;

					if (HeatSensor::detectVictim(HeatSensorSide::left)) {
						if (tempFusedData.distSensorState.leftFront == DistSensorStatus::underflow ||
							(tempFusedData.distSensorState.leftFront == DistSensorStatus::ok && tempFusedData.distances.leftFront / 10.0f < 20.0f)) {
							consecutiveLHeat++;
						}
						else {
							consecutiveLHeat = 0;
						}
					}
					else {
						consecutiveLHeat = 0;
					}

					if (HeatSensor::detectVictim(HeatSensorSide::right)) {
						if (tempFusedData.distSensorState.rightFront == DistSensorStatus::underflow ||
							(tempFusedData.distSensorState.rightFront == DistSensorStatus::ok && tempFusedData.distances.rightFront / 10.0f < 20.0f)) {
							consecutiveRHeat++;
						}
						else {
							consecutiveRHeat = 0;
						}
					}
					else {
						consecutiveRHeat = 0;
					}

					if (consecutiveLHeat >= 4 && lastHeatVictim != tempFusedData.robotState.mapCoordinate)
						leftHeat = true;
					else if (consecutiveRHeat >= 4 && lastHeatVictim != tempFusedData.robotState.mapCoordinate)
						rightHeat = true;

					if (rightHeat || leftHeat) {
						lastHeatVictim = tempFusedData.robotState.mapCoordinate;

						GridCell currentCell;
						MazeMapping::getGridCell(&currentCell, tempFusedData.robotState.mapCoordinate);

						if (!(currentCell.cellState & CellState::victim)) {
							SmoothDriving::stop();

							Serial.println("start disp");
							auto startDisp = millis();

							currentCell.cellState |= CellState::victim;
							MazeMapping::setGridCell(currentCell, tempFusedData.robotState.mapCoordinate);

							if (leftHeat) {
								Dispenser::dispenseLeft(1);
							}
							else {
								Dispenser::dispenseRight(1);
							}

							while (millis() - startDisp < 6000) {
								PowerLEDs::setBrightness(sinf(millis() / 200.0f) * 0.5f + 0.5f);
							}

							PowerLEDs::setBrightness(JAFDSettings::PowerLEDs::defaultPower);

							leftHeat = false;
							rightHeat = false;
						}

						return;
					}

					static uint16_t consLeftOneCol = 0;
					static uint16_t consLeftZeroCol = 0;

					if (CamRec::getVictim(true) == Victim::red || CamRec::getVictim(true) == Victim::yellow) {
						consLeftOneCol++;
						consLeftZeroCol = 0;
					}
					else if (CamRec::getVictim(true) == Victim::green) {
						consLeftZeroCol++;
						consLeftOneCol = 0;
					}
					else {
						consLeftZeroCol = 0;
						consLeftOneCol = 0;
					}

					static uint16_t consRightOneCol = 0;
					static uint16_t consRightZeroCol = 0;

					if (CamRec::getVictim(false) == Victim::red || CamRec::getVictim(false) == Victim::yellow) {
						consRightOneCol++;
						consRightZeroCol = 0;
					}
					else if (CamRec::getVictim(false) == Victim::green) {
						consRightZeroCol++;
						consRightOneCol = 0;
					}
					else {
						consRightZeroCol = 0;
						consRightOneCol = 0;
					}

					static MapCoordinate lastColorVictim = homePosition;
					if ((consRightZeroCol >= 2 || consRightOneCol >= 2 || consLeftZeroCol >= 2 || consLeftOneCol >= 2) && lastColorVictim != tempFusedData.robotState.mapCoordinate) {
						if (consLeftZeroCol >= 2 || consLeftOneCol >= 2) {
							if (tempFusedData.distSensorState.leftBack == DistSensorStatus::ok && tempFusedData.distances.leftBack < 200) {
							}
							else {
								consLeftZeroCol = 0;
								consLeftOneCol = 0;
								return;
							}
						}
						else {
							if (tempFusedData.distSensorState.rightBack == DistSensorStatus::ok && tempFusedData.distances.rightBack < 200) {
							}
							else {
								consRightZeroCol = 0;
								consRightOneCol = 0;
								return;
							}
						}

						if (SmoothDriving::isDrivingStraight()) {
							if (!SmoothDriving::isTaskFinished()) {
								return;
							}
						}

						lastColorVictim = tempFusedData.robotState.mapCoordinate;

						GridCell currentCell;
						MazeMapping::getGridCell(&currentCell, tempFusedData.robotState.mapCoordinate);

						if (!(currentCell.cellState & CellState::victim)) {
							SmoothDriving::stop();

							Serial.println("start disp");
							auto startDisp = millis();

							currentCell.cellState |= CellState::victim;
							MazeMapping::setGridCell(currentCell, tempFusedData.robotState.mapCoordinate);

							if (consLeftOneCol >= 2) {
								Dispenser::dispenseLeft(1);
							}
							else if (consRightOneCol >= 2) {
								Dispenser::dispenseRight(1);
							}

							consRightZeroCol = 0;
							consRightOneCol = 0;
							consLeftZeroCol = 0;
							consLeftOneCol = 0;

							while (millis() - startDisp < 6000) {
								PowerLEDs::setBrightness(sinf(millis() / 200.0f) * 0.5f + 0.5f);
							}

							PowerLEDs::setBrightness(JAFDSettings::PowerLEDs::defaultPower);
						}

						return;
					}
				}
		*/

		void loop()
		{
			using namespace SmoothDriving;

			static MapCoordinate lastCoordinate = homePosition;

			static bool start = true;
			static bool startAfterRotation = false;
			static uint8_t cumSureWalls = 0b0000;
			static bool rejectCellChange = false;	// if true -> reject a cell position change -> hysteresis like effect

			FusedData fusedData = SensorFusion::getFusedData();

			if (getInformation().finished) {
				rejectCellChange = false;
			}

			if (start) {
				if (!SensorFusion::scanSurrounding(cumSureWalls)) {
					return;
				}

				SensorFusion::updatePosAndRotFromDist(1000);

				fusedData = SensorFusion::getFusedData();

				if (fusedData.gridCell.cellConnections & EntranceDirections::east) {
					Serial.println("start rot. east");
					startNewTask(new TaskArray({ new Rotate(-M_PI_2, -2.0f), new AlignWalls() }));
				}
				else {
					Serial.println("start rot. west");
					startNewTask(new TaskArray({ new Rotate(M_PI_2, 2.0f), new AlignWalls() }));
				}

				start = false;
				startAfterRotation = true;
			}
			else if (startAfterRotation) {
				if (!getInformation().finished) {
					return;
				}

				if (!SensorFusion::scanSurrounding(cumSureWalls)) {
					return;
				}

				Serial.println("startAfterRotation -> updatePos");

				fusedData = SensorFusion::getFusedData();
				Serial.println(fusedData.gridCell.cellConnections, BIN);
				startFirstTask(fusedData);

				startAfterRotation = false;
			}
			else {/*
				static bool returnFromBlack = false;

				if (isDrivingStraight()) {
					FloorTileColour tileColour = ColorSensor::detectTileColour(fusedData.colorSensData.lux);

					if (tileColour == FloorTileColour::black && !returnFromBlack) {
						Serial.println("start return from black");
						startNewTask<NewStateType::lastEndState>(TaskArray(FollowWall(-25, -30.0f), AlignWalls(), Stop()), true);
						returnFromBlack = true;
						return;
					}
				}
				*/
				if (SmoothDriving::getInformation().finished) {
					//if (returnFromBlack) {
					//	GridCell cell;
					//	MazeMapping::getGridCell(&cell, fusedData.robotState.mapCoordinate.getCoordinateInDir(fusedData.robotState.heading));
					//	cell.cellState = cell.cellState | CellState::blackTile;
					//	MazeMapping::setGridCell(cell, fusedData.robotState.mapCoordinate.getCoordinateInDir(fusedData.robotState.heading));
					//	Serial.println("store black tile");
					//	returnFromBlack = false;
					//}

					if (SensorFusion::scanSurrounding(cumSureWalls)) {
						Serial.println("scanned surr. -> update pos");
						SensorFusion::updatePosAndRotFromDist();

						fusedData = SensorFusion::getFusedData();

						RelativeDir driveDir;
						if (getNextDrivingDir(fusedData, driveDir)) {
							Serial.print("start new turn dir. drive: ");
							Serial.println((int)driveDir);
							startRelativeTurnDirDrive(driveDir, fusedData);
						}
					}
				}
			}

			if (fusedData.robotState.mapCoordinate != lastCoordinate && !rejectCellChange) {
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

				rejectCellChange = true;
			}

			lastCoordinate = fusedData.robotState.mapCoordinate;
		}
	}
}