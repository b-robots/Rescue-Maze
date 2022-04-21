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
#include "../header/Math.h"
#include "../header/SmallThings.h"

namespace JAFD
{
	namespace RobotLogic
	{
		namespace {
			void startRelativeTurnDirDrive(RelativeDir dir, FusedData tempFusedData) {
				using namespace SmoothDriving;

				RobotState newStartState;

				newStartState.wheelSpeeds = FloatWheelSpeeds{ 0.0f, 0.0f };
				newStartState.forwardVel = static_cast<float>(0.0f);
				newStartState.angularVel = Vec3f(0.0f, 0.0f, 0.0f);

				float heading = fitAngleToInterval(tempFusedData.robotState.globalHeading);

				uint8_t entranceDirs = EntranceDirections::nowhere;

				if (heading > M_PI_4 && heading < M_3PI_4) {
					newStartState.globalHeading = makeRotationCoherent(tempFusedData.robotState.globalHeading, M_PI_2);
				}
				else if (heading < -M_PI_4 && heading > -M_3PI_4) {
					newStartState.globalHeading = makeRotationCoherent(tempFusedData.robotState.globalHeading, -M_PI_2);
				}
				else if (heading < M_PI_4 && heading > -M_PI_4) {
					newStartState.globalHeading = makeRotationCoherent(tempFusedData.robotState.globalHeading, 0);
				}
				else {
					newStartState.globalHeading = makeRotationCoherent(tempFusedData.robotState.globalHeading, M_PI);
				}

				newStartState.position = Vec3f(tempFusedData.robotState.mapCoordinate.x * 30.0f, tempFusedData.robotState.mapCoordinate.y * 30.0f, 0.0f);

				switch (dir)
				{
				case RelativeDir::forward:
					setNewTask(TaskArray(Stop(),
						FollowWall(25, 30.0f),
						AlignWalls(),
						Stop()), newStartState);
					break;
				case RelativeDir::right:
					setNewTask(TaskArray(Stop(),
						Rotate(-1.0f, -90.0f),
						AlignWalls(),
						FollowWall(25, 30.0f),
						AlignWalls(),
						Stop()), newStartState);
					break;
				case RelativeDir::backward:
					setNewTask(TaskArray(Stop(),
						Rotate(1.0f, 180.0f),
						AlignWalls(),
						FollowWall(25, 30.0f),
						AlignWalls(),
						Stop()), newStartState);
					break;
				case RelativeDir::left:
					setNewTask(TaskArray(Stop(),
						Rotate(1.0f, 90.0f),
						AlignWalls(),
						FollowWall(25, 30.0f),
						AlignWalls(),
						Stop()), newStartState);
					break;
				default:
					break;
				}
			}
			volatile DistSensorStates _triggeredStates;
			volatile Distances _triggeredDistances;
		}

		// Start task which prefers straigth over left over right over back
		// Intended for the first maneuver after starting
		void startFirstTask(FusedData tempFusedData) {
			AbsoluteDir absLeft = makeAbsolute(RelativeDir::left, tempFusedData.robotState.heading);
			AbsoluteDir absForward = makeAbsolute(RelativeDir::forward, tempFusedData.robotState.heading);
			AbsoluteDir absRight = makeAbsolute(RelativeDir::right, tempFusedData.robotState.heading);
			AbsoluteDir absBack = makeAbsolute(RelativeDir::backward, tempFusedData.robotState.heading);

			if (tempFusedData.gridCell.cellConnections & static_cast<uint8_t>(absForward)) {
				startRelativeTurnDirDrive(RelativeDir::forward, tempFusedData);
			}
			else
			{
				if (tempFusedData.gridCell.cellConnections & static_cast<uint8_t>(absLeft) && tempFusedData.gridCell.cellConnections & static_cast<uint8_t>(absRight))
				{
					if (random(2) == 0) {
						Serial.println("start first left");
						startRelativeTurnDirDrive(RelativeDir::left, tempFusedData);
					}
					else {
						Serial.println("start first right");
						startRelativeTurnDirDrive(RelativeDir::right, tempFusedData);
					}
				}
				else if (tempFusedData.gridCell.cellConnections & static_cast<uint8_t>(absLeft)) {
					Serial.println("start first left");
					startRelativeTurnDirDrive(RelativeDir::left, tempFusedData);
				}
				else if (tempFusedData.gridCell.cellConnections & static_cast<uint8_t>(absRight)) {
					Serial.println("start first right");
					startRelativeTurnDirDrive(RelativeDir::right, tempFusedData);
				}
				else if (tempFusedData.gridCell.cellConnections & static_cast<uint8_t>(absBack)) {
					Serial.println("start first back");
					startRelativeTurnDirDrive(RelativeDir::backward, tempFusedData);
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

			// Check for black & stair tile
			if (possibleRelDir.L) {
				MazeMapping::getGridCell(&cell, tempFusedData.robotState.mapCoordinate.getCoordinateInDir(absLeft));
				if (cell.cellState & CellState::blackTile || cell.cellState & CellState::stair) {
					possibleRelDir.L = false;
				}
			}

			if (possibleRelDir.R) {
				MazeMapping::getGridCell(&cell, tempFusedData.robotState.mapCoordinate.getCoordinateInDir(absRight));
				if (cell.cellState & CellState::blackTile || cell.cellState & CellState::stair) {
					possibleRelDir.R = false;
				}
			}

			if (possibleRelDir.F) {
				MazeMapping::getGridCell(&cell, tempFusedData.robotState.mapCoordinate.getCoordinateInDir(absForward));
				if (cell.cellState & CellState::blackTile || cell.cellState & CellState::stair) {
					possibleRelDir.F = false;
				}
			}

			if (possibleRelDir.B) {
				MazeMapping::getGridCell(&cell, tempFusedData.robotState.mapCoordinate.getCoordinateInDir(absBack));
				if (cell.cellState & CellState::blackTile || cell.cellState & CellState::stair) {
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

			// Select possible dir randomly
			if (possibleRelDir.F || possibleRelDir.L || possibleRelDir.R) {
				while (true) {
					int i = random(2);

					if (i == 0 && possibleRelDir.F) {
						nextDir = RelativeDir::forward;
						return true;
					}
					else if (i == 1 && possibleRelDir.L) {
						nextDir = RelativeDir::left;
						return true;
					}
					else if (i == 1 && possibleRelDir.R) {
						nextDir = RelativeDir::right;
						return true;
					}
				}
			}
			else if (possibleRelDir.B) {
				nextDir = RelativeDir::backward;
			}
			else {
				nextDir = RelativeDir::backward;
				Serial.println("No direction to drive!");
				// return false;
			}

			return true;
		}

		void handleVictimDetection(FusedData tempFusedData) {
			// TODO

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

			if (consecutiveLHeat > 5 && lastHeatVictim != tempFusedData.robotState.mapCoordinate)
				leftHeat = true;
			else if (consecutiveRHeat > 5 && lastHeatVictim != tempFusedData.robotState.mapCoordinate)
				rightHeat = true;

			if (rightHeat || leftHeat) {
				lastHeatVictim = tempFusedData.robotState.mapCoordinate;

				GridCell currentCell;
				MazeMapping::getGridCell(&currentCell, tempFusedData.robotState.mapCoordinate);

				if (!(currentCell.cellState & CellState::victim)) {
					SmoothDriving::stopTask();

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
						PowerLEDs::setBrightness(sinf(millis() / 500.0f) * 0.5f + 0.5f);
					}

					PowerLEDs::setBrightness(JAFDSettings::PowerLEDs::defaultPower);

					leftHeat = false;
					rightHeat = false;
				}
			}
		}

		void triggerFrontDistSnapshot() {
			auto tempFusedData = SensorFusion::getFusedData();
			_triggeredStates = tempFusedData.distSensorState;
			_triggeredDistances = tempFusedData.distances;
		}

		void loop()
		{
			using namespace SmoothDriving;

			static MapCoordinate lastCoordinate = homePosition;

			static bool start = true;
			static bool startAfterRotation = false;
			static bool ramp = false;
			static bool stairs = false;
			static uint8_t cumSureWalls = 0b0000;
			static bool blockCellChange = false;

			FusedData tempFusedData = SensorFusion::getFusedData();

			if (isTaskFinished()) {
				blockCellChange = false;
			}

			static RobotState endStateRamp;
			if (tempFusedData.gridCell.cellState & CellState::ramp && !ramp && !stairs && isDrivingStraight()) {
				bool detectedRamp = false;

				if (_triggeredStates.frontLeft == DistSensorStatus::underflow ||
					_triggeredStates.frontRight == DistSensorStatus::underflow) {
					detectedRamp = true;
				}
				else {
					if (_triggeredStates.frontLeft == DistSensorStatus::ok &&
						_triggeredDistances.frontLeft < 300.0f) {
						detectedRamp = true;
					}

					if (_triggeredStates.frontRight == DistSensorStatus::ok &&
						_triggeredDistances.frontRight < 300.0f) {
						detectedRamp = true;
					}

					if (tempFusedData.robotState.pitch < 0.0f) {
						detectedRamp = true;
					}
				}

				endStateRamp = getEndState();

				//if (!detectedRamp) {
				//	Serial.println("return from stairs");
				//	setNewTask<NewStateType::lastEndState>(ReturnStairs(-20), true);
				//	stairs = true;
				//	return;
				//}

				// Do stuff for ramp
				Serial.println("ramp detected -> ramp task");
				setNewTask<NewStateType::lastEndState>(Ramp(tempFusedData.robotState.pitch > 0.0f ? 30 : 20), true);
				ramp = true;

				int8_t mapX = roundf(endStateRamp.position.x / 30.0f);
				int8_t mapY = roundf(endStateRamp.position.y / 30.0f);

				float heading = fitAngleToInterval(endStateRamp.globalHeading);

				if (heading > M_PI_4 && heading < M_3PI_4) {
					mapY--;
				}
				else if (heading < -M_PI_4 && heading > -M_3PI_4) {
					mapY++;
				}
				else if (heading < M_PI_4 && heading > -M_PI_4) {
					mapX--;
				}
				else {
					mapX++;
				}

				Serial.print("left cell (on ramp) - x: ");
				Serial.print(mapX);
				Serial.print(", y: ");
				Serial.print(mapY);
				Serial.print(", walls: ");
				Serial.println(cumSureWalls, BIN);

				GridCell currCell;
				MazeMapping::getGridCell(&currCell, MapCoordinate(mapX, mapY));
				currCell.cellConnections = (currCell.cellConnections & ~CellConnections::directionMask) | ~cumSureWalls;
				currCell.cellState |= CellState::visited;
				MazeMapping::setGridCell(currCell, MapCoordinate(mapX, mapY));
				cumSureWalls = 0b0000;
				blockCellChange = true;
			}

			if (start) {
				if (!SensorFusion::scanSurrounding(cumSureWalls)) {
					return;
				}

				SensorFusion::updatePosAndRotFromDist();

				tempFusedData = SensorFusion::getFusedData();

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

				tempFusedData = SensorFusion::getFusedData();
				Serial.println(tempFusedData.gridCell.cellConnections, BIN);
				startFirstTask(tempFusedData);

				startAfterRotation = false;
			}
			else if (ramp) {
				if (isTaskFinished()) {
					blockCellChange = true;

					RobotState newStartState;
					NewForcedFusionValues correctedState;

					correctedState.zeroPitch = true;
					correctedState.newHeading = true;
					correctedState.newX = true;
					correctedState.newY = true;
					correctedState.clearCell = true;

					newStartState.wheelSpeeds = FloatWheelSpeeds{ 0.0f, 0.0f };
					newStartState.forwardVel = static_cast<float>(0.0f);
					newStartState.angularVel = Vec3f(0.0f, 0.0f, 0.0f);

					int16_t mapX = roundf(endStateRamp.position.x / 30.0f);
					int16_t mapY = roundf(endStateRamp.position.y / 30.0f);

					float heading = fitAngleToInterval(endStateRamp.globalHeading);

					uint8_t entranceDirs = EntranceDirections::nowhere;

					if (heading > M_PI_4 && heading < M_3PI_4) {
						newStartState.globalHeading = makeRotationCoherent(tempFusedData.robotState.globalHeading, M_PI_2);
						mapY++;
						entranceDirs |= EntranceDirections::west;
					}
					else if (heading < -M_PI_4 && heading > -M_3PI_4) {
						newStartState.globalHeading = makeRotationCoherent(tempFusedData.robotState.globalHeading, -M_PI_2);
						mapY--;
						entranceDirs |= EntranceDirections::east;
					}
					else if (heading < M_PI_4 && heading > -M_PI_4) {
						newStartState.globalHeading = makeRotationCoherent(tempFusedData.robotState.globalHeading, 0);
						mapX++;
						entranceDirs |= EntranceDirections::north;
					}
					else {
						newStartState.globalHeading = makeRotationCoherent(tempFusedData.robotState.globalHeading, M_PI);
						mapX--;
						entranceDirs |= EntranceDirections::south;
					}

					newStartState.position = Vec3f(mapX * 30.0f, mapY * 30.0f, 0.0f);

					correctedState.heading = newStartState.globalHeading;
					correctedState.x = newStartState.position.x;
					correctedState.y = newStartState.position.y;

					GridCell cell;
					cell.cellConnections = entranceDirs;
					cell.cellState = CellState::visited | CellState::ramp;
					MazeMapping::setGridCell(cell, MapCoordinate(roundf(endStateRamp.position.x / 30.0f), roundf(endStateRamp.position.y / 30.0f)));

					Serial.print("stored ramp cell - x: ");
					Serial.print(roundf(endStateRamp.position.x / 30.0f));
					Serial.print(", y: ");
					Serial.print(roundf(endStateRamp.position.y / 30.0f));
					Serial.print(", connections: ");
					Serial.println(entranceDirs, BIN);

					SensorFusion::setCorrectedState(correctedState);
					setNewTask(TaskArray(Stop(), AlignWalls()), newStartState);
					cumSureWalls = 0b0000;

					ramp = false;
				}

				return;
			}
			else if (stairs) {
				if (isTaskFinished()) {
					Serial.println("finished retreating stairs");

					GridCell cell;
					cell.cellConnections = 0b0000;
					cell.cellState = CellState::visited | CellState::stair;
					MazeMapping::setGridCell(cell, MapCoordinate(roundf(endStateRamp.position.x / 30.0f), roundf(endStateRamp.position.y / 30.0f)));

					RobotState newStartState;

					newStartState.wheelSpeeds = FloatWheelSpeeds{ 0.0f, 0.0f };
					newStartState.forwardVel = static_cast<float>(0.0f);
					newStartState.angularVel = Vec3f(0.0f, 0.0f, 0.0f);

					int16_t mapX = roundf(endStateRamp.position.x / 30.0f);
					int16_t mapY = roundf(endStateRamp.position.y / 30.0f);

					float heading = fitAngleToInterval(endStateRamp.globalHeading);

					if (heading > M_PI_4 && heading < M_3PI_4) {
						newStartState.globalHeading = makeRotationCoherent(tempFusedData.robotState.globalHeading, M_PI_2);
						mapY--;
					}
					else if (heading < -M_PI_4 && heading > -M_3PI_4) {
						newStartState.globalHeading = makeRotationCoherent(tempFusedData.robotState.globalHeading, -M_PI_2);
						mapY++;
					}
					else if (heading < M_PI_4 && heading > -M_PI_4) {
						newStartState.globalHeading = makeRotationCoherent(tempFusedData.robotState.globalHeading, 0);
						mapX--;
					}
					else {
						newStartState.globalHeading = makeRotationCoherent(tempFusedData.robotState.globalHeading, M_PI);
						mapX++;
					}

					newStartState.position = Vec3f(mapX * 30.0f, mapY * 30.0f, 0.0f);

					NewForcedFusionValues correctedState;
					correctedState.clearCell = true;
					SensorFusion::setCorrectedState(correctedState);

					setNewTask(TaskArray(Stop(), AlignWalls()), newStartState);

					stairs = false;
				}
				return;
			}
			else {
				if (SensorFusion::getConsRotStuck() > 10) {
					Serial.println("rot stuck");
					stopTask();
				}

				handleVictimDetection(tempFusedData);

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

						tempFusedData = SensorFusion::getFusedData();

						RelativeDir driveDir;
						if (getNextDrivingDir(tempFusedData, driveDir)) {
							Serial.print("start new turn dir. drive: ");
							Serial.println((int)driveDir);
							startRelativeTurnDirDrive(driveDir, tempFusedData);
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

		void timeBetweenUpdate(bool blink)
		{
			if (blink) {
				PowerLEDs::setBrightness(sinf(millis() / 500) * 0.5f + 0.5f);
			}

			CamRec::loop();
		}
	}

}