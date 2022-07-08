#include "../header/RobotLogic.h"
#include "../header/SensorFusion.h"
#include "../header/MazeMapping.h"
#include "../header/SmoothDriving.h"
#include "../header/MotorControl.h"
#include "../header/HeatSensor.h"
#include "../header/Dispenser.h"
#include "../header/ColorSensor.h"
#include "../header/Math.h"
#include "../header/SmallThings.h"
#include "../header/CamRec.h"
#include "../header/DistanceSensors.h"
#include "../header/Gyro.h"

namespace SIAL
{
	namespace RobotLogic
	{
		namespace {
			uint8_t imaginaryWalls = 0b0000;
		}

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
					new Rotate(-M_PI_2, -3.0f),
					new AlignWalls(),
					new FollowCell(30),
					new AlignWalls() }));
				break;
			case RelativeDir::backward:
				startNewTask(new TaskArray({
					new Rotate(M_PI, 3.0f),
					new AlignWalls(),
					new FollowCell(30),
					new AlignWalls() }));
				break;
			case RelativeDir::left:
				startNewTask(new TaskArray({
					new Rotate(M_PI_2, 3.0f),
					new AlignWalls(),
					new FollowCell(30),
					new AlignWalls() }));
				break;
			default:
				break;
			}
		}

		bool startFirstTask(FusedData fusedData) {
			AbsoluteDir absLeft = makeAbsolute(RelativeDir::left, fusedData.robotState.heading);
			AbsoluteDir absForward = makeAbsolute(RelativeDir::forward, fusedData.robotState.heading);
			AbsoluteDir absRight = makeAbsolute(RelativeDir::right, fusedData.robotState.heading);
			AbsoluteDir absBack = makeAbsolute(RelativeDir::backward, fusedData.robotState.heading);

			if (fusedData.gridCell.cellConnections & static_cast<uint8_t>(absForward)) {
				startRelativeTurnDirDrive(RelativeDir::forward, fusedData);
			}
			else
			{
				if (fusedData.gridCell.cellConnections & static_cast<uint8_t>(absLeft)) {
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
					return false;
				}
			}

			return true;
		}

		bool getNextDrivingDir(FusedData fusedData, RelativeDir& nextDir) {
			fusedData.gridCell.cellConnections = ~(~fusedData.gridCell.cellConnections | imaginaryWalls);
			imaginaryWalls = 0b0000;

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

			static AbsoluteDir pathToFollow[128];
			static uint8_t pathIndex = 0;
			static uint8_t pathLength = 0;
			static bool followPath = false;
			static MapCoordinate lastPathPosition = MapCoordinate(INT8_MAX, INT8_MAX);
			static uint8_t consPathNotPossible = 0;
			static bool driveHome = false;

			Serial.print("Get new direction based on cell (WSEN): ");
			Serial.println(fusedData.gridCell.cellConnections, BIN);

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

			if (pathIndex+1 >= pathLength && pathLength > 0) {
				lastPathPosition = MapCoordinate(INT8_MAX, INT8_MAX);
				pathIndex = 0;
				pathLength = 0;
				followPath = false;
				consPathNotPossible = 0;
				Serial.println("Finished path");

				if (fusedData.robotState.mapCoordinate == homePosition && driveHome) {
					Serial.println("Found home!");

					while (true) {
						// TODO -> maybe allow LOP if this is not the correct spot
					}
				}
			}

			if (followPath) {
				if (lastPathPosition != fusedData.robotState.mapCoordinate) {
					pathIndex++;
				}
				lastPathPosition = fusedData.robotState.mapCoordinate;

				Serial.println("Follow path");
				RelativeDir nextPathDir = makeRelative(pathToFollow[pathIndex], fusedData.robotState.heading);
				Serial.println((int)nextPathDir);

				bool notPossible = true;
				switch (nextPathDir)
				{
				case SIAL::RelativeDir::forward:
					if (possibleRelDir.F) {
						nextDir = RelativeDir::forward;
						notPossible = false;
					}
					break;
				case SIAL::RelativeDir::backward:
					if (possibleRelDir.B) {
						nextDir = RelativeDir::backward;
						notPossible = false;
					}
					break;
				case SIAL::RelativeDir::left:
					if (possibleRelDir.L) {
						nextDir = RelativeDir::left;
						notPossible = false;
					}
					break;
				case SIAL::RelativeDir::right:
					if (possibleRelDir.R) {
						nextDir = RelativeDir::right;
						notPossible = false;
					}
					break;
				default:
					break;
				}

				if (notPossible) {
					Serial.println("Path is not possible!");
					pathIndex = 0;
					pathLength = 0;
					followPath = false;
					consPathNotPossible++;
					return false;
				}

				consPathNotPossible = 0;
				return true;
			}

			if (possibleUnvisitedRelDir.L) {
				nextDir = RelativeDir::left;
			}
			else if (possibleUnvisitedRelDir.F) {
				nextDir = RelativeDir::forward;
			}
			else if (possibleUnvisitedRelDir.R) {
				nextDir = RelativeDir::right;
			}
			else if (possibleUnvisitedRelDir.B) {
				nextDir = RelativeDir::backward;
			}
			else {
				Serial.println("No unvisited field found; Search for nearest unvisited field...");

				// Set cell (needed for path finding) 
				GridCell startCell;
				MazeMapping::getGridCell(&startCell, fusedData.robotState.mapCoordinate);
				startCell.cellConnections = fusedData.gridCell.cellConnections;
				startCell.cellState |= CellState::visited;
				MazeMapping::setGridCell(startCell, fusedData.robotState.mapCoordinate);

				if (consPathNotPossible >= 2 ||
					MazeMapping::BFAlgorithm::findShortestPath(fusedData.robotState.mapCoordinate, pathToFollow, 128, pathLength,
						MazeMapping::BFAlgorithm::GoalFunctions::unvisitedNeighbour) != ReturnCode::ok) {
					Serial.println("No path findable!");
					if (MazeMapping::returnToHome()) {
						Serial.println("Everything recovered!!");
						Serial.println("Return home...");
						if (MazeMapping::BFAlgorithm::findShortestPath(fusedData.robotState.mapCoordinate, pathToFollow, 128, pathLength,
							MazeMapping::BFAlgorithm::GoalFunctions::home) == ReturnCode::ok) {
							pathIndex = 0;
							followPath = true;

							RelativeDir nextPathDir = makeRelative(pathToFollow[pathIndex], fusedData.robotState.heading);
							Serial.println((int)nextPathDir);

							bool notPossible = true;
							switch (nextPathDir)
							{
							case SIAL::RelativeDir::forward:
								if (possibleRelDir.F) {
									nextDir = RelativeDir::forward;
									notPossible = false;
								}
								break;
							case SIAL::RelativeDir::backward:
								if (possibleRelDir.B) {
									nextDir = RelativeDir::backward;
									notPossible = false;
								}
								break;
							case SIAL::RelativeDir::left:
								if (possibleRelDir.L) {
									nextDir = RelativeDir::left;
									notPossible = false;
								}
								break;
							case SIAL::RelativeDir::right:
								if (possibleRelDir.R) {
									nextDir = RelativeDir::right;
									notPossible = false;
								}
								break;
							default:
								break;
							}

							if (notPossible) {
								Serial.println("Path is not possible!");
								pathIndex = 0;
								pathLength = 0;
								followPath = false;
								consPathNotPossible++;
								return false;
							}

							lastPathPosition = fusedData.robotState.mapCoordinate;
							consPathNotPossible = 0;

							driveHome = true;

							return true;
						}
						else {
							Serial.println("Can't find home!");
						}
					}

					if (possibleRelDir.L) {
						nextDir = RelativeDir::left;
					}
					else if (possibleRelDir.F) {
						nextDir = RelativeDir::forward;
					}
					else if (possibleRelDir.R) {
						nextDir = RelativeDir::right;
					}
					else if (possibleRelDir.B) {
						nextDir = RelativeDir::backward;
					}
					else {
						if (possibleOnlyWallsRelDir.L + possibleOnlyWallsRelDir.F + possibleOnlyWallsRelDir.R >= 2) {
							Serial.println("Choose next dir randomly");
							bool found = false;
							while (!found) {
								auto r = random(3);
								if (r == 0 && possibleOnlyWallsRelDir.L) {
									nextDir = RelativeDir::left;
									found = true;
								}
								else if (r == 1 && possibleOnlyWallsRelDir.R) {
									nextDir = RelativeDir::right;
									found = true;
								}
								else if (possibleOnlyWallsRelDir.F) {
									nextDir = RelativeDir::forward;
									found = true;
								}
							}
						}
						else if (possibleOnlyWallsRelDir.L) {
							nextDir = RelativeDir::left;
						}
						else if (possibleOnlyWallsRelDir.F) {
							nextDir = RelativeDir::forward;
						}
						else if (possibleOnlyWallsRelDir.R) {
							nextDir = RelativeDir::right;
						}
						else if (possibleOnlyWallsRelDir.B) {
							nextDir = RelativeDir::backward;
						}
						else {
							Serial.println("No direction to drive!");
							return false;
						}
					}
				}
				else {
					pathIndex = 0;
					followPath = true;

					RelativeDir nextPathDir = makeRelative(pathToFollow[pathIndex], fusedData.robotState.heading);
					Serial.println((int)nextPathDir);

					bool notPossible = true;
					switch (nextPathDir)
					{
					case SIAL::RelativeDir::forward:
						if (possibleRelDir.F) {
							nextDir = RelativeDir::forward;
							notPossible = false;
						}
						break;
					case SIAL::RelativeDir::backward:
						if (possibleRelDir.B) {
							nextDir = RelativeDir::backward;
							notPossible = false;
						}
						break;
					case SIAL::RelativeDir::left:
						if (possibleRelDir.L) {
							nextDir = RelativeDir::left;
							notPossible = false;
						}
						break;
					case SIAL::RelativeDir::right:
						if (possibleRelDir.R) {
							nextDir = RelativeDir::right;
							notPossible = false;
						}
						break;
					default:
						break;
					}

					if (notPossible) {
						Serial.println("Path is not possible!");
						pathIndex = 0;
						pathLength = 0;
						followPath = false;
						consPathNotPossible++;
						return false;
					}
					
					lastPathPosition = fusedData.robotState.mapCoordinate;
					consPathNotPossible = 0;

					driveHome = false;

					return true;
				}
			}

			consPathNotPossible = 0;
			return true;
		}

		void handleHeatVictim(FusedData fusedData) {
			static uint32_t lastHeatDetection = 0;
			static int consecutiveLHeat = 0;
			static int consecutiveRHeat = 0;
			bool leftHeat = false;
			bool rightHeat = false;
			static MapCoordinate lastHeatVictim = homePosition;

			float lookAtAngle;
			if (fusedData.robotState.globalHeading - floorf(fusedData.robotState.globalHeading / M_PI_2) * M_PI_2 < M_PI_4) {
				lookAtAngle = floorf(fusedData.robotState.globalHeading / M_PI_2) * M_PI_2;
			}
			else {
				lookAtAngle = ceilf(fusedData.robotState.globalHeading / M_PI_2) * M_PI_2;
			}

			static uint32_t lastLeftFrontRisingEdge = 0;
			if (fusedData.fusedDistSens.risingEdge.leftFront) {
				lastLeftFrontRisingEdge = millis();
			}

			static uint32_t lastRightFrontRisingEdge = 0;
			if (fusedData.fusedDistSens.risingEdge.rightFront) {
				lastRightFrontRisingEdge = millis();
			}

			if (HeatSensor::detectVictim(HeatSensorSide::left)) {
				if (SmoothDriving::getInformation().uid == DrivingTaskUID::rotate) {
					float angle = fitAngleToInterval(lookAtAngle + M_PI_2);
					AbsoluteDir wallToCheck;

					if (angle > M_PI_4 && angle < M_3PI_4) {
						wallToCheck = AbsoluteDir::west;
					}
					else if (angle < -M_PI_4 && angle > -M_3PI_4) {
						wallToCheck = AbsoluteDir::east;
					}
					else if (angle < M_PI_4 && angle > -M_PI_4) {
						wallToCheck = AbsoluteDir::north;
					}
					else {
						wallToCheck = AbsoluteDir::south;
					}

					if (!(fusedData.gridCell.cellConnections & (uint8_t)wallToCheck)) {
						consecutiveLHeat++;
					}
					else {
						if (fabsf(fitAngleToInterval(fusedData.robotState.globalHeading * 4.0f) / 4.0f) < 20.0f * DEG_TO_RAD) {
							if ((fusedData.distSensorState.leftFront == DistSensorStatus::ok ||
								fusedData.distSensorState.leftFront == DistSensorStatus::underflow) &&
								(fusedData.distSensorState.leftBack == DistSensorStatus::ok ||
									fusedData.distSensorState.leftBack == DistSensorStatus::underflow))
							{
								consecutiveLHeat++;
							}
							else {
								consecutiveLHeat = 0;
							}
						}
						else {
							consecutiveLHeat = 0;
						}
					}
				}
				else if (SmoothDriving::getInformation().uid == DrivingTaskUID::alignWalls) {
					if ((fusedData.distSensorState.leftFront == DistSensorStatus::ok ||
						fusedData.distSensorState.leftFront == DistSensorStatus::underflow) &&
						(fusedData.distSensorState.leftBack == DistSensorStatus::ok ||
							fusedData.distSensorState.leftBack == DistSensorStatus::underflow))
					{
						consecutiveLHeat++;
					}
					else {
						consecutiveLHeat = 0;
					}
				}
				else if (SmoothDriving::getInformation().uid == DrivingTaskUID::followCell) {
					if ((fusedData.distSensorState.leftFront == DistSensorStatus::ok ||
						fusedData.distSensorState.leftFront == DistSensorStatus::underflow) &&
						millis() - lastLeftFrontRisingEdge > 5.0f / (30.0f / 1000.0f)) {
						consecutiveLHeat++;
					}
					else {
						consecutiveLHeat = 0;
					}
				}
				else {
					consecutiveLHeat = 0;
				}
			}
			else {
				consecutiveLHeat = 0;
			}

			if (HeatSensor::detectVictim(HeatSensorSide::right)) {
				if (SmoothDriving::getInformation().uid == DrivingTaskUID::rotate) {
					float angle = fitAngleToInterval(lookAtAngle - M_PI_2);
					AbsoluteDir wallToCheck;

					if (angle > M_PI_4 && angle < M_3PI_4) {
						wallToCheck = AbsoluteDir::west;
					}
					else if (angle < -M_PI_4 && angle > -M_3PI_4) {
						wallToCheck = AbsoluteDir::east;
					}
					else if (angle < M_PI_4 && angle > -M_PI_4) {
						wallToCheck = AbsoluteDir::north;
					}
					else {
						wallToCheck = AbsoluteDir::south;
					}

					if (!(fusedData.gridCell.cellConnections & (uint8_t)wallToCheck)) {
						consecutiveRHeat++;
					}
					else {
						if (fabsf(fitAngleToInterval(fusedData.robotState.globalHeading * 4.0f) / 4.0f) < 20.0f * DEG_TO_RAD) {
							if ((fusedData.distSensorState.rightFront == DistSensorStatus::ok ||
								fusedData.distSensorState.rightFront == DistSensorStatus::underflow) &&
								(fusedData.distSensorState.rightBack == DistSensorStatus::ok ||
									fusedData.distSensorState.rightBack == DistSensorStatus::underflow))
							{
								consecutiveRHeat++;
							}
							else {
								consecutiveRHeat = 0;
							}
						}
						else {
							consecutiveRHeat = 0;
						}
					}
				}
				else if (SmoothDriving::getInformation().uid == DrivingTaskUID::alignWalls) {
					if ((fusedData.distSensorState.rightFront == DistSensorStatus::ok ||
						fusedData.distSensorState.rightFront == DistSensorStatus::underflow) &&
						(fusedData.distSensorState.rightBack == DistSensorStatus::ok ||
							fusedData.distSensorState.rightBack == DistSensorStatus::underflow))
					{
						consecutiveRHeat++;
					}
					else {
						consecutiveRHeat = 0;
					}
				}
				else if (SmoothDriving::getInformation().uid == DrivingTaskUID::followCell) {
					if ((fusedData.distSensorState.rightFront == DistSensorStatus::ok ||
						fusedData.distSensorState.rightFront == DistSensorStatus::underflow) &&
						millis() - lastRightFrontRisingEdge > 5.0f / (30.0f / 1000.0f)) {
						consecutiveRHeat++;
					}
					else {
						consecutiveRHeat = 0;
					}
				}
				else {
					consecutiveRHeat = 0;
				}
			}
			else {
				consecutiveRHeat = 0;
			}

			if (millis() - lastHeatDetection < 500) {
				consecutiveLHeat = consecutiveRHeat = 0;
			}

			MapCoordinate victimCoordinate = fusedData.robotState.mapCoordinate;

			switch (fusedData.robotState.heading)
			{
			case AbsoluteDir::north:
				victimCoordinate.x = roundf((fusedData.robotState.position.x + 4.0f) / 30.0f);
				break;
			case AbsoluteDir::east:
				victimCoordinate.y = roundf((fusedData.robotState.position.y - 4.0f) / 30.0f);
				break;
			case AbsoluteDir::south:
				victimCoordinate.x = roundf((fusedData.robotState.position.x - 4.0f) / 30.0f);
				break;
			case AbsoluteDir::west:
				victimCoordinate.y = roundf((fusedData.robotState.position.y + 4.0f) / 30.0f);
				break;
			default:
				break;
			}

			if (consecutiveLHeat >= 3 && lastHeatVictim != victimCoordinate) {
				leftHeat = true;
				Serial.println("heat victim detected -> left");
			}
			else if (consecutiveRHeat >= 3 && lastHeatVictim != victimCoordinate) {
				rightHeat = true;
				Serial.println("heat victim detected -> right");
			}

			if (rightHeat || leftHeat) {
				Serial.print("Victim at x: ");
				Serial.print(victimCoordinate.x);
				Serial.print(", y: ");
				Serial.println(victimCoordinate.y);

				lastHeatVictim = victimCoordinate;

				GridCell currentCell;
				MazeMapping::getGridCell(&currentCell, victimCoordinate);

				if (currentCell.cellState & (CellState::blackTile || CellState::checkpoint)) {
					consecutiveLHeat = 0;
					consecutiveRHeat = 0;
					return;
				}

				if (!(currentCell.cellState & CellState::victim)) {
					float goalDist = NAN;

					RobotState taskStartState = SmoothDriving::getInformation().startState;

					if (SmoothDriving::getInformation().uid == DrivingTaskUID::rotate) {
						Serial.println("rotate to straight position");
						SmoothDriving::startNewTask(new SmoothDriving::TaskArray{ new SmoothDriving::Rotate((lookAtAngle - fusedData.robotState.globalHeading) * 0.9f, sgn(lookAtAngle - fusedData.robotState.globalHeading) * 2.0f), new SmoothDriving::AlignWalls() }, true);
						while (!SmoothDriving::getInformation().finished) {
							SensorFusion::updateSensors();
							SensorFusion::distSensFusion();
							SensorFusion::sensorFusion();
							SmoothDriving::updateSpeeds();
						}
					}
					else if (SmoothDriving::getInformation().uid == DrivingTaskUID::followCell) {
						switch (fusedData.robotState.heading)
						{
						case SIAL::AbsoluteDir::north:
							goalDist = taskStartState.position.x + 30.0f - fusedData.robotState.position.x;
							break;
						case SIAL::AbsoluteDir::east:
							goalDist = -(taskStartState.position.y - 30.0f - fusedData.robotState.position.y);
							break;
						case SIAL::AbsoluteDir::south:
							goalDist = -(taskStartState.position.x - 30.0f - fusedData.robotState.position.x);
							break;
						case SIAL::AbsoluteDir::west:
							goalDist = taskStartState.position.y + 30.0f - fusedData.robotState.position.y;
							break;
						default:
							break;
						}
					}

					SmoothDriving::stop();

					Serial.println("start disp");
					auto startDisp = millis();

					currentCell.cellState |= CellState::victim;
					MazeMapping::setGridCell(currentCell, victimCoordinate);

					if (leftHeat) {
						Dispenser::dispenseLeft(1);
					}
					else {
						Dispenser::dispenseRight(1);
					}

					while (millis() - startDisp < 5500) {
						SensorFusion::sensorFusion();

						PowerLEDs::setBrightness(sinf(millis() / SIALSettings::PowerLEDs::blinkPeriod * M_TWOPI) / 2.0f + 0.5f);
					}

					PowerLEDs::setBrightness(SIALSettings::PowerLEDs::defaultPower);

					if (!std::isnan(goalDist)) {
						Serial.println("finish cell");

						SmoothDriving::startNewTask(new SmoothDriving::TaskArray{ new SmoothDriving::FollowWall(goalDist * 0.9f, 30), new SmoothDriving::AlignWalls() }, true);
					}

					consecutiveLHeat = 0;
					consecutiveRHeat = 0;
				}
			}
		}

		void handleCamVictim(FusedData fusedData) {
			static uint16_t consLeftOneCol = 0;
			static uint16_t consLeftZeroCol = 0;
			static uint16_t consLeftH = 0;
			static uint16_t consLeftS = 0;
			static uint16_t consLeftU = 0;
			static uint16_t consRightOneCol = 0;
			static uint16_t consRightZeroCol = 0;
			static uint16_t consRightH = 0;
			static uint16_t consRightS = 0;
			static uint16_t consRightU = 0;

			GridCell currentCell;
			MazeMapping::getGridCell(&currentCell, fusedData.robotState.mapCoordinate);

			if (currentCell.cellState & CellState::victim) {
				consLeftOneCol = 0;
				consLeftZeroCol = 0;
				consRightOneCol = 0;
				consRightZeroCol = 0;
			}

			bool leftValidWall = false;

			if (leftValidWall) {
				if (CamRec::getVictim(true) == Victim::red || CamRec::getVictim(true) == Victim::yellow) {
					consLeftOneCol++;
					consLeftZeroCol = 0;
					consLeftH = 0;
					consLeftS = 0;
					consLeftU = 0;
				}
				else if (CamRec::getVictim(true) == Victim::green) {
					consLeftOneCol = 0;
					consLeftZeroCol++;
					consLeftH = 0;
					consLeftS = 0;
					consLeftU = 0;
				}
				else if (CamRec::getVictim(true) == Victim::harmed) {
					consLeftOneCol = 0;
					consLeftZeroCol = 0;
					consLeftH++;
					consLeftS = 0;
					consLeftU = 0;
				}
				else if (CamRec::getVictim(true) == Victim::stable) {
					consLeftOneCol = 0;
					consLeftZeroCol = 0;
					consLeftH = 0;
					consLeftS++;
					consLeftU = 0;
				}
				else if (CamRec::getVictim(true) == Victim::unharmed) {
					consLeftOneCol = 0;
					consLeftZeroCol = 0;
					consLeftH = 0;
					consLeftS = 0;
					consLeftU++;
				}
				else {
					consLeftOneCol = 0;
					consLeftZeroCol = 0;
					consLeftH = 0;
					consLeftS = 0;
					consLeftU = 0;
				}
			}
			else {
				consLeftOneCol = 0;
				consLeftZeroCol = 0;
				consLeftH = 0;
				consLeftS = 0;
				consLeftU = 0;
			}

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
			if ((consRightZeroCol >= 2 || consRightOneCol >= 2 || consLeftZeroCol >= 2 || consLeftOneCol >= 2) && lastColorVictim != fusedData.robotState.mapCoordinate) {
				if (consLeftZeroCol >= 2 || consLeftOneCol >= 2) {
					if (fusedData.distSensorState.leftBack == DistSensorStatus::ok && fusedData.distances.leftBack < 200) {
					}
					else {
						consLeftZeroCol = 0;
						consLeftOneCol = 0;
						return;
					}
				}
				else {
					if (fusedData.distSensorState.rightBack == DistSensorStatus::ok && fusedData.distances.rightBack < 200) {
					}
					else {
						consRightZeroCol = 0;
						consRightOneCol = 0;
						return;
					}
				}

				lastColorVictim = fusedData.robotState.mapCoordinate;

				GridCell currentCell;
				MazeMapping::getGridCell(&currentCell, fusedData.robotState.mapCoordinate);

				if (!(currentCell.cellState & CellState::victim)) {
					SmoothDriving::stop();

					Serial.println("start disp");
					auto startDisp = millis();

					currentCell.cellState |= CellState::victim;
					MazeMapping::setGridCell(currentCell, fusedData.robotState.mapCoordinate);

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

					while (millis() - startDisp < 5500) {
						SensorFusion::sensorFusion();

						PowerLEDs::setBrightness(sinf(millis() / SIALSettings::PowerLEDs::blinkPeriod * M_TWOPI) / 2.0f + 0.5f);
					}

					PowerLEDs::setBrightness(SIALSettings::PowerLEDs::defaultPower);
				}

				return;
			}
		}

		void handleBumper() {
			const auto retreat = []() {
				SmoothDriving::startNewTask(new SmoothDriving::TaskArray{
					new SmoothDriving::ForceSpeed(-5, -20),
					new SmoothDriving::Rotate(0, 1.0f)
					}, true);

				while (!SmoothDriving::getInformation().finished) {
					Gyro::updateValues();
					SensorFusion::sensorFusion();
					SmoothDriving::updateSpeeds();
				}
			};

			auto fusedData = SensorFusion::getFusedData();

			static MapCoordinate lastBumperCoor = MapCoordinate(INT8_MAX, INT8_MAX);
			static AbsoluteDir lastBumperDir = AbsoluteDir::north;
			static uint8_t consBumper = 0;

			const auto checkConsBumper = [fusedData]() {
				if (consBumper >= 1) {
					Serial.println("Too many consecutive bumper trigger... Set imaginary wall");
					imaginaryWalls = (uint8_t)fusedData.robotState.heading;
				}
			};

			const auto uid = SmoothDriving::getInformation().uid;
			if (uid == DrivingTaskUID::stop) {
				const auto startWait = millis();
				if (Bumper::leftInterrupt || Bumper::rightInterrupt) {
					if (lastBumperCoor == fusedData.robotState.mapCoordinate &&
						lastBumperDir == fusedData.robotState.heading) {
						consBumper++;
					}
					else {
						consBumper = 0;
					}

					lastBumperCoor = fusedData.robotState.mapCoordinate;
					lastBumperDir = fusedData.robotState.heading;

					checkConsBumper();

					while (millis() - startWait < 10) {
						if (Bumper::leftInterrupt && Bumper::rightInterrupt) {
							Serial.println("Both bumper triggered");
							retreat();
							Bumper::leftInterrupt = false;
							Bumper::rightInterrupt = false;
							return;
						}
					}

					while (!SensorFusion::waitForAllDistSens()) {
						DistanceSensors::updateDistSensors();
					}

					fusedData = SensorFusion::getFusedData();

					bool distSensOk = false;
					if (Bumper::leftInterrupt) {
						if (fusedData.distSensorState.frontRight == DistSensorStatus::overflow ||
							(fusedData.distSensorState.frontRight == DistSensorStatus::ok && fusedData.distances.frontRight > 250)) {
							distSensOk = true;

							if (fusedData.distSensorState.frontRight == DistSensorStatus::ok && fusedData.distances.frontRight > 320) {
								if (fusedData.distSensorState.frontLong == DistSensorStatus::underflow) {
									distSensOk = false;
								}
							}
						}
					}
					else {
						if (fusedData.distSensorState.frontLeft == DistSensorStatus::overflow ||
							(fusedData.distSensorState.frontLeft == DistSensorStatus::ok && fusedData.distances.frontLeft > 250)) {
							distSensOk = true;

							if (fusedData.distSensorState.frontLeft == DistSensorStatus::ok && fusedData.distances.frontLeft > 320) {
								if (fusedData.distSensorState.frontLeft == DistSensorStatus::underflow) {
									distSensOk = false;
								}
							}
						}
					}

					if (distSensOk) {
						Serial.println("Side bumper triggered -> side step");

						// Only one bumper triggered -> side step
						SmoothDriving::startNewTask(new SmoothDriving::TaskArray{
							new SmoothDriving::SideStep(Bumper::rightInterrupt),
							new SmoothDriving::ForceSpeed(-3, -20),
							new SmoothDriving::Rotate(0, 1.0f) }, true);

						Bumper::leftInterrupt = false;
						Bumper::rightInterrupt = false;

						while (!SmoothDriving::getInformation().finished) {
							Gyro::updateValues();
							SensorFusion::sensorFusion();
							SmoothDriving::updateSpeeds();
						}

						return;
					}
					else {
						Serial.println("Bumper triggered + distance sensors see a wall...");
						retreat();
					}
				}
			}
			else if (SmoothDriving::getInformation().finished) {
				if (Bumper::isPressed(true) || Bumper::isPressed(false)) {
					if (lastBumperCoor == fusedData.robotState.mapCoordinate &&
						lastBumperDir == fusedData.robotState.heading) {
						consBumper++;
					}
					else {
						consBumper = 0;
					}

					lastBumperCoor = fusedData.robotState.mapCoordinate;
					lastBumperDir = fusedData.robotState.heading;

					checkConsBumper();

					Serial.println("Both bumper triggered");
					retreat();
				}
			}

			Bumper::leftInterrupt = false;
			Bumper::rightInterrupt = false;
		}

		void loop()
		{
			using namespace SmoothDriving;

			static MapCoordinate lastCoordinate = homePosition;

			static MapCoordinate lastStoredCell = MapCoordinate(INT8_MAX, INT8_MAX);

			static bool start = true;
			static bool startAfterRotation = false;
			static uint8_t cumSureWalls = 0b0000;
			static bool rejectCellChange = false;	// if true -> reject a cell position change
			static DrivingTaskUID lastTask = DrivingTaskUID::invalid;
			static uint8_t consSilver = 0;
			static MapCoordinate lastCkpt = homePosition;

			static DistSensorStates frontSnapshotStates;
			static Distances frontSnapshotDistances;

			FusedData fusedData = SensorFusion::getFusedData();

			if (getInformation().finished) {
				rejectCellChange = false;
			}

			if (start) {
				if (!SensorFusion::waitForAllDistSens()) {
					return;
				}

				if (!SensorFusion::scanSurrounding(cumSureWalls)) {
					return;
				}

				SensorFusion::updatePosAndRotFromDist(1000);

				fusedData = SensorFusion::getFusedData();

				if (fusedData.gridCell.cellConnections & EntranceDirections::west) {
					Serial.println("start rot. west");
					startNewTask(new TaskArray({ new Rotate(M_PI_2, 3.0f), new AlignWalls() }));
				}
				else {
					Serial.println("start rot. east");
					startNewTask(new TaskArray({ new Rotate(-M_PI_2, -3.0f), new AlignWalls() }));
				}

				start = false;
				startAfterRotation = true;
			}
			else if (startAfterRotation) {
				if (!getInformation().finished) {
					return;
				}

				if (!SensorFusion::waitForAllDistSens()) {
					return;
				}

				if (!SensorFusion::scanSurrounding(cumSureWalls)) {
					return;
				}

				Serial.println("startAfterRotation -> updatePos");

				fusedData = SensorFusion::getFusedData();
				if (!startFirstTask(fusedData)) {
					cumSureWalls = 0b0000;

					NewForcedFusionValues correctedState;
					correctedState.clearCell = true;
					SensorFusion::setCorrectedState(correctedState);

					return;
				}

				startAfterRotation = false;
			}
			else {
				auto robotState = fusedData.robotState;

				// Stuck during rotation handling
				if (SensorFusion::consecutiveRotStuck > 60) {
					Serial.println("Stuck during rotation!");

					startNewTask(new RotationUnstuck(), true);
				}

				handleBumper();

				// Ramp and stair handling
				static uint8_t consPosIncline = 0;
				static uint8_t consNegIncline = 0;
				static bool rampUp = false;
				static MapCoordinate rampPos;

				if (SmoothDriving::getInformation().drivingStraight) {
					if (robotState.forwardVel > SIALSettings::MotorControl::minSpeed && robotState.pitch > SIALSettings::MazeMapping::minRampAngle * (consPosIncline > 0 ? 0.8f : 1.0f)) {
						consPosIncline++;
					}
					else {
						consPosIncline = 0;
					}

					if (robotState.forwardVel > SIALSettings::MotorControl::minSpeed && robotState.pitch < -SIALSettings::MazeMapping::minRampAngle * (consNegIncline > 0 ? 0.8f : 1.0f)) {
						consNegIncline++;
					}
					else {
						consNegIncline = 0;
					}

					if (consPosIncline >= 20) {
						if (frontSnapshotStates.frontLong == DistSensorStatus::underflow ||
							frontSnapshotStates.frontLong == DistSensorStatus::ok && frontSnapshotDistances.frontLong < (30.0f + 15.0f - SIALSettings::Mechanics::distSensFrontBackDist / 2.0f) * 10) {
							fusedData.gridCell.cellState |= CellState::ramp;
							rampUp = true;
						}
						else {
							fusedData.gridCell.cellState |= CellState::stair;
						}
					}
					else if (consNegIncline >= 10) {
						if (rampUp) {
							fusedData.gridCell.cellState |= CellState::ramp;
							rampUp = false;
						}
					}
				}
				else {
					consPosIncline = 0;
					consNegIncline = 0;
				}

				bool updateLastCellOnRamp = false;
				if ((fusedData.gridCell.cellState & CellState::ramp) && SmoothDriving::getInformation().uid != DrivingTaskUID::ramp && SmoothDriving::getInformation().uid != DrivingTaskUID::stairs) {
					Serial.println(rampUp ? "ramp up" : "ramp down");
					startNewTask(new Ramp(rampUp ? 30 : 20, rampUp), true);
					updateLastCellOnRamp = true;
				}
				else if ((fusedData.gridCell.cellState & CellState::stair) && SmoothDriving::getInformation().uid != DrivingTaskUID::ramp && SmoothDriving::getInformation().uid != DrivingTaskUID::stairs) {
					Serial.println("stair");
					startNewTask(new Stairs(20), true);
					updateLastCellOnRamp = true;
				}

				if (updateLastCellOnRamp) {
					// Set cell information of last cell
					int8_t mapX = robotState.mapCoordinate.x;
					int8_t mapY = robotState.mapCoordinate.y;

					rampPos = robotState.mapCoordinate;

					switch (robotState.heading) {
					case AbsoluteDir::north:
						mapX = roundf((robotState.position.x - 15.0f) / 30.0f);
						rampPos.x = mapX + 1;
						break;
					case AbsoluteDir::east:
						mapY = roundf((robotState.position.y + 15.0f) / 30.0f);
						rampPos.y = mapY - 1;
						break;
					case AbsoluteDir::south:
						mapX = roundf((robotState.position.x + 15.0f) / 30.0f);
						rampPos.x = mapX - 1;
						break;
					case AbsoluteDir::west:
						mapY = roundf((robotState.position.y - 15.0f) / 30.0f);
						rampPos.y = mapY + 1;
						break;
					default:
						break;
					}

					if (MapCoordinate(mapX, mapY) != lastStoredCell) {
						Serial.print("left cell on ramp/stairs - x: ");
						Serial.print(mapX);
						Serial.print(", y: ");
						Serial.print(mapY);
						Serial.print(", walls: ");
						Serial.println(cumSureWalls, BIN);

						GridCell currCell;
						MazeMapping::getGridCell(&currCell, MapCoordinate(mapX, mapY));
						currCell.cellConnections = ~cumSureWalls;
						currCell.cellState |= CellState::visited;
						MazeMapping::setGridCell(currCell, MapCoordinate(mapX, mapY));
						cumSureWalls = 0b0000;

						lastStoredCell = MapCoordinate(mapX, mapY);
					}

					rejectCellChange = true;
				}

				// Tile handling
				static bool returnFromBlack = false;

				if (SmoothDriving::getInformation().drivingStraight && SmoothDriving::getInformation().uid != DrivingTaskUID::ramp && SmoothDriving::getInformation().uid != DrivingTaskUID::stairs && fabsf(fusedData.robotState.pitch) < 0.05f) {
					FloorTileColour tileColour = ColorSensor::detectTileColour(fusedData.colorSensData.lux);

					if (tileColour == FloorTileColour::black && !returnFromBlack) {
						Serial.println("start return from black");

						int32_t dist;
						switch (robotState.heading)
						{
						case AbsoluteDir::north:
							dist = floorf(robotState.position.x / 30.0f) * 30.0f - robotState.position.x;
							break;
						case AbsoluteDir::east:
							dist = robotState.position.y - ceilf(robotState.position.y / 30.0f) * 30.0f;
							break;
						case AbsoluteDir::south:
							dist = robotState.position.x - ceilf(robotState.position.x / 30.0f) * 30.0f;
							break;
						case AbsoluteDir::west:
							dist = floorf(robotState.position.y / 30.0f) * 30.0f - robotState.position.y;
							break;
						default:
							break;
						}

						startNewTask(new TaskArray{ new FollowWall(dist, -40.0f), new AlignWalls() }, true);
						returnFromBlack = true;
						rejectCellChange = true;

						return;
					}

					if (tileColour == FloorTileColour::silver) {
						consSilver++;
					}
					else {
						consSilver = 0;
					}
				}
				else {
					consSilver = 0;
				}

				if (consSilver >= 5 && lastCkpt != fusedData.robotState.mapCoordinate) {
					consSilver = 0;

					int8_t mapX = robotState.mapCoordinate.x;
					int8_t mapY = robotState.mapCoordinate.y;

					switch (robotState.heading)
					{
					case AbsoluteDir::north:
						mapX = roundf((robotState.position.x + SIALSettings::Mechanics::robotLength / 2.0f) / 30.0f);
						break;
					case AbsoluteDir::east:
						mapY = roundf((robotState.position.y - SIALSettings::Mechanics::robotLength / 2.0f) / 30.0f);
						break;
					case AbsoluteDir::south:
						mapX = roundf((robotState.position.x - SIALSettings::Mechanics::robotLength / 2.0f) / 30.0f);
						break;
					case AbsoluteDir::west:
						mapY = roundf((robotState.position.y + SIALSettings::Mechanics::robotLength / 2.0f) / 30.0f);
						break;
					default:
						break;
					}

					lastCkpt = MapCoordinate(mapX, mapY);

					GridCell cell;
					MazeMapping::getGridCell(&cell, lastCkpt);
					cell.cellState |= CellState::checkpoint;
					MazeMapping::setGridCell(cell, lastCkpt);
					Serial.print("stored checkpoint - x: ");
					Serial.print(mapX);
					Serial.print(", y: ");
					Serial.println(mapY);
				}

				handleHeatVictim(fusedData);
				handleCamVictim(fusedData);

				if (SmoothDriving::getInformation().finished) {
					if (SmoothDriving::getInformation().uid == DrivingTaskUID::ramp) {
						NewForcedFusionValues correctedState;

						correctedState.zeroPitch = true;
						correctedState.newX = true;
						correctedState.newY = true;
						correctedState.clearCell = true;

						uint8_t entranceDirs = EntranceDirections::nowhere;

						switch (robotState.heading) {
						case AbsoluteDir::north:
						case AbsoluteDir::south:
							entranceDirs = EntranceDirections::north | EntranceDirections::south;
							break;
						case AbsoluteDir::east:
						case AbsoluteDir::west:
							entranceDirs = EntranceDirections::east | EntranceDirections::west;
							break;
						default:
							break;
						}

						correctedState.x = rampPos.getCoordinateInDir(robotState.heading).x * 30.0f;
						correctedState.y = rampPos.getCoordinateInDir(robotState.heading).y * 30.0f;

						GridCell cell;
						cell.cellConnections = entranceDirs;
						cell.cellState = CellState::visited | CellState::ramp;
						MazeMapping::setGridCell(cell, rampPos);

						Serial.print("stored ramp cell - x: ");
						Serial.print(rampPos.x);
						Serial.print(", y: ");
						Serial.print(rampPos.y);
						Serial.print(", connections: ");
						Serial.println(entranceDirs, BIN);

						SensorFusion::setCorrectedState(correctedState);
						SmoothDriving::startNewTask(new AlignWalls());

						cumSureWalls = 0b0000;
						rejectCellChange = true;
					}
					else if (SmoothDriving::getInformation().uid == DrivingTaskUID::stairs) {
						float stairLen = 0.0f;
						uint8_t lenEstimates = 0;

						if (fusedData.distSensorState.frontLong == DistSensorStatus::ok) {
							stairLen += fmaxf((frontSnapshotDistances.frontLong - fusedData.distances.frontLong) / 10.0f, 0.0f);
							lenEstimates++;
						}

						if (fusedData.distSensorState.frontLeft == DistSensorStatus::ok) {
							stairLen += fmaxf((frontSnapshotDistances.frontLong - fusedData.distances.frontLeft) / 10.0f, 0.0f);
							lenEstimates++;
						}
						else if (fusedData.distSensorState.frontLeft == DistSensorStatus::underflow) {
							stairLen += fmaxf((frontSnapshotDistances.frontLong - DistanceSensors::VL53L0::minDist) / 10.0f, 0.0f);
							lenEstimates++;
						}

						if (fusedData.distSensorState.frontRight == DistSensorStatus::ok) {
							stairLen += fmaxf((frontSnapshotDistances.frontLong - fusedData.distances.frontRight) / 10.0f, 0.0f);
							lenEstimates++;
						}
						else if (fusedData.distSensorState.frontRight == DistSensorStatus::underflow) {
							stairLen += fmaxf((frontSnapshotDistances.frontLong - DistanceSensors::VL53L0::minDist) / 10.0f, 0.0f);
							lenEstimates++;
						}

						stairLen /= lenEstimates;
						uint8_t numStairs = roundf(stairLen / 30.0f) - 1;

						Serial.print("finished: ");
						Serial.print(numStairs);
						Serial.println(" stairs");

						NewForcedFusionValues correctedState;

						correctedState.zeroPitch = true;
						correctedState.newX = true;
						correctedState.newY = true;
						correctedState.clearCell = true;

						uint8_t entranceDirs = EntranceDirections::nowhere;

						switch (robotState.heading) {
						case AbsoluteDir::north:
						case AbsoluteDir::south:
							entranceDirs = EntranceDirections::north | EntranceDirections::south;
							break;
						case AbsoluteDir::east:
						case AbsoluteDir::west:
							entranceDirs = EntranceDirections::east | EntranceDirections::west;
							break;
						default:
							break;
						}

						for (int i = 0; i < numStairs; i++) {
							GridCell cell;
							cell.cellConnections = entranceDirs;
							cell.cellState = CellState::visited | CellState::stair;
							MazeMapping::setGridCell(cell, rampPos);

							Serial.print("stored stair cell - x: ");
							Serial.print(rampPos.x);
							Serial.print(", y: ");
							Serial.print(rampPos.y);
							Serial.print(", connections: ");
							Serial.println(entranceDirs, BIN);

							rampPos = rampPos.getCoordinateInDir(robotState.heading);
						}

						correctedState.x = rampPos.x * 30.0f;
						correctedState.y = rampPos.y * 30.0f;

						SensorFusion::setCorrectedState(correctedState);
						SmoothDriving::startNewTask(new AlignWalls());

						cumSureWalls = 0b0000;
						rejectCellChange = true;
					}

					if (!SensorFusion::waitForAllDistSens()) {
						return;
					}

					if (returnFromBlack) {
						GridCell cell;
						MazeMapping::getGridCell(&cell, fusedData.robotState.mapCoordinate.getCoordinateInDir(fusedData.robotState.heading));
						cell.cellState = CellState::blackTile;
						MazeMapping::setGridCell(cell, fusedData.robotState.mapCoordinate.getCoordinateInDir(fusedData.robotState.heading));
						Serial.print("stored black tile: ");
						Serial.print(fusedData.robotState.mapCoordinate.getCoordinateInDir(fusedData.robotState.heading).x);
						Serial.print(", ");
						Serial.println(fusedData.robotState.mapCoordinate.getCoordinateInDir(fusedData.robotState.heading).y);
						returnFromBlack = false;
					}

					if (SensorFusion::scanSurrounding(cumSureWalls)) {
						Serial.println("scanned surr. -> update pos");
						SensorFusion::updatePosAndRotFromDist();

						fusedData = SensorFusion::getFusedData();
						robotState = fusedData.robotState;

						RelativeDir driveDir;
						if (getNextDrivingDir(fusedData, driveDir)) {
							Serial.print("start new turn dir. drive: ");
							Serial.println((int)driveDir);
							startRelativeTurnDirDrive(driveDir, fusedData);
						}
						else {
							cumSureWalls = 0b0000;

							NewForcedFusionValues correctedState;
							correctedState.clearCell = true;
							SensorFusion::setCorrectedState(correctedState);

							return;
						}
					}
				}
			}

			if (getInformation().uid == DrivingTaskUID::followCell && lastTask != DrivingTaskUID::followCell) {
				frontSnapshotStates = DistSensorStates();
				frontSnapshotStates.frontLeft = fusedData.distSensorState.frontLeft;
				frontSnapshotStates.frontLong = fusedData.distSensorState.frontLong;
				frontSnapshotStates.frontRight = fusedData.distSensorState.frontRight;

				frontSnapshotDistances = Distances();
				frontSnapshotDistances.frontLeft = fusedData.distances.frontLeft;
				frontSnapshotDistances.frontLong = fusedData.distances.frontLong;
				frontSnapshotDistances.frontRight = fusedData.distances.frontRight;
				Serial.println("forward driving start");
			}

			if (fusedData.robotState.mapCoordinate != lastCoordinate && !rejectCellChange && lastStoredCell != lastCoordinate) {
				Serial.print("left cell - x: ");
				Serial.print(lastCoordinate.x);
				Serial.print(", y: ");
				Serial.print(lastCoordinate.y);
				Serial.print(", walls: ");
				Serial.println(cumSureWalls, BIN);

				GridCell currCell;
				MazeMapping::getGridCell(&currCell, lastCoordinate);
				currCell.cellConnections = ~cumSureWalls;
				currCell.cellState |= CellState::visited;
				MazeMapping::setGridCell(currCell, lastCoordinate);
				cumSureWalls = 0b0000;

				lastStoredCell = lastCoordinate;

				consSilver = 0;

				rejectCellChange = true;
			}

			lastCoordinate = fusedData.robotState.mapCoordinate;
			lastTask = getInformation().uid;
		}
	}
}