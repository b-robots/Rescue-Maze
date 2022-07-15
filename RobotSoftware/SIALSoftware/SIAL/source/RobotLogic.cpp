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
			uint32_t startTime = 0;

			bool heatVictim = false;

			uint8_t imaginaryWalls = 0b0000;
		}

		void startRelativeTurnDirDrive(RelativeDir dir, FusedData fusedData) {
			using namespace SmoothDriving;

			switch (dir)
			{
			case RelativeDir::forward:
				startNewTask(new TaskArray({
					new FollowCell(38),
					new AlignWalls() }));
				break;
			case RelativeDir::right:
				startNewTask(new TaskArray({
					new Rotate(-M_PI_2, -3.0f),
					new AlignWalls(),
					new FollowCell(38),
					new AlignWalls() }));
				break;
			case RelativeDir::backward:
				startNewTask(new TaskArray({
					new Rotate(M_PI, 3.0f),
					new AlignWalls(),
					new FollowCell(38),
					new AlignWalls() }));
				break;
			case RelativeDir::left:
				startNewTask(new TaskArray({
					new Rotate(M_PI_2, 3.0f),
					new AlignWalls(),
					new FollowCell(38),
					new AlignWalls() }));
				break;
			default:
				break;
			}
		}

		bool getNextDrivingDir(FusedData fusedData, RelativeDir& nextDir) {
			fusedData.gridCell.cellConnections = ~((~fusedData.gridCell.cellConnections) | imaginaryWalls);
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

			if (possibleRelDir.F) {
				nextDir = RelativeDir::forward;
			}
			else if (possibleRelDir.L) {
				nextDir = RelativeDir::left;
			}
			else {
				nextDir = RelativeDir::right;
			}

			return true;
		}

		void handleHeatVictim(FusedData fusedData) {
			static uint32_t lastHeatDetection = 0;
			static int consecutiveLHeat = 0;
			static int consecutiveRHeat = 0;
			bool leftHeat = false;
			bool rightHeat = false;
			static MapCoordinate lastHeatVictim = homePosition;

			if (fabsf(fusedData.robotState.pitch) > 0.1) {
				consecutiveLHeat = 0;
				consecutiveRHeat = 0;
				return;
			}

			if (HeatSensor::detectVictim(HeatSensorSide::left)) {
				if (SmoothDriving::getInformation().uid == DrivingTaskUID::rotate) {
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
						(fusedData.distSensorState.leftBack == DistSensorStatus::ok ||
							fusedData.distSensorState.leftBack == DistSensorStatus::underflow)) {
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
						(fusedData.distSensorState.rightBack == DistSensorStatus::ok ||
							fusedData.distSensorState.rightBack == DistSensorStatus::underflow)) {
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

			if (consecutiveLHeat >= 1 && lastHeatVictim != victimCoordinate) {
				leftHeat = true;
				Serial.println("heat victim detected -> left");
			}
			else if (consecutiveRHeat >= 1 && lastHeatVictim != victimCoordinate) {
				rightHeat = true;
				Serial.println("heat victim detected -> right");
			}

			if (rightHeat || leftHeat) {
				Serial.print("Victim at x: ");
				Serial.print(victimCoordinate.x);
				Serial.print(", y: ");
				Serial.println(victimCoordinate.y);

				lastHeatVictim = victimCoordinate;

				heatVictim = true;

				RobotState taskStartState = SmoothDriving::getInformation().startState;
				auto taskUid = SmoothDriving::getInformation().uid;

				bool finishRotateRight = false;
				fusedData = SensorFusion::getFusedData();
				if (fusedData.robotState.wheelSpeeds.left > fusedData.robotState.wheelSpeeds.right) {
					finishRotateRight = true;
				}

				SmoothDriving::stop();

				auto startDisp = millis();
				while (millis() - startDisp < 5500) {
					SensorFusion::sensorFusion();

					PowerLEDs::setBrightness(sinf(millis() / SIALSettings::PowerLEDs::blinkPeriod * M_TWOPI) / 2.0f + 0.5f);
				}

				PowerLEDs::setBrightness(SIALSettings::PowerLEDs::defaultPower);

				if (taskUid == DrivingTaskUID::rotate) {
					float goalAngle = fusedData.robotState.globalHeading;

					if (finishRotateRight) {
						goalAngle = floorf(goalAngle / M_PI_2) * M_PI_2;
					}
					else {
						goalAngle = ceilf(goalAngle / M_PI_2) * M_PI_2;
					}

					SmoothDriving::startNewTask(new SmoothDriving::Rotate(goalAngle - fusedData.robotState.globalHeading, sgn(goalAngle - fusedData.robotState.globalHeading) * 2.0f));
				}

				consecutiveLHeat = 0;
				consecutiveRHeat = 0;
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

			static MapCoordinate lastVisVictim = homePosition;
			static uint32_t lastVisDetection = 0;

			if (heatVictim) {
				heatVictim = false;
				consLeftOneCol = 0;
				consLeftZeroCol = 0;
				consLeftH = 0;
				consLeftS = 0;
				consLeftU = 0;
				consRightOneCol = 0;
				consRightZeroCol = 0;
				consRightH = 0;
				consRightS = 0;
				consRightU = 0;
				return;
			}

			GridCell currentCell;
			MazeMapping::getGridCell(&currentCell, fusedData.robotState.mapCoordinate);

			if ((currentCell.cellState & CellState::victim) ||
				(currentCell.cellState & CellState::checkpoint) ||
				(currentCell.cellState & CellState::blackTile) ||
				lastVisVictim == fusedData.robotState.mapCoordinate ||
				fabsf(fusedData.robotState.pitch) > 0.1 ||
				millis() - lastVisDetection < 500) {
				consLeftOneCol = 0;
				consLeftZeroCol = 0;
				consLeftH = 0;
				consLeftS = 0;
				consLeftU = 0;
				consRightOneCol = 0;
				consRightZeroCol = 0;
				consRightH = 0;
				consRightS = 0;
				consRightU = 0;

				return;
			}

			bool leftWallValid = false;
			bool rightWallValid = false;

			float lookAtAngle;
			if (fusedData.robotState.globalHeading - floorf(fusedData.robotState.globalHeading / M_PI_2) * M_PI_2 < M_PI_4) {
				lookAtAngle = floorf(fusedData.robotState.globalHeading / M_PI_2) * M_PI_2;
			}
			else {
				lookAtAngle = ceilf(fusedData.robotState.globalHeading / M_PI_2) * M_PI_2;
			}

			if (!CamRec::dataReady) {
				return;
			}

			if (SmoothDriving::getInformation().uid == DrivingTaskUID::followCell ||
				SmoothDriving::getInformation().uid == DrivingTaskUID::alignWalls) {
				if (fusedData.distSensorState.leftBack == DistSensorStatus::underflow ||
					fusedData.distSensorState.leftBack == DistSensorStatus::ok) {
					if (fusedData.distSensorState.leftFront == DistSensorStatus::underflow ||
						fusedData.distSensorState.leftFront == DistSensorStatus::ok) {
						leftWallValid = true;
					}
				}

				if (fusedData.distSensorState.rightBack == DistSensorStatus::underflow ||
					fusedData.distSensorState.rightBack == DistSensorStatus::ok) {
					if (fusedData.distSensorState.rightFront == DistSensorStatus::underflow ||
						fusedData.distSensorState.rightFront == DistSensorStatus::ok) {
						rightWallValid = true;
					}
				}
			}
			else if (SmoothDriving::getInformation().uid == DrivingTaskUID::rotate) {
				if ((fusedData.distSensorState.leftFront == DistSensorStatus::ok ||
					fusedData.distSensorState.leftFront == DistSensorStatus::underflow) &&
					(fusedData.distSensorState.leftBack == DistSensorStatus::ok ||
						fusedData.distSensorState.leftBack == DistSensorStatus::underflow))
				{
					leftWallValid = true;
				}

				if ((fusedData.distSensorState.rightFront == DistSensorStatus::ok ||
					fusedData.distSensorState.rightFront == DistSensorStatus::underflow) &&
					(fusedData.distSensorState.rightBack == DistSensorStatus::ok ||
						fusedData.distSensorState.rightBack == DistSensorStatus::underflow))
				{
					rightWallValid = true;
				}
				else {
					rightWallValid = false;
				}
			}

			if (leftWallValid) {
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

			if (rightWallValid) {
				if (CamRec::getVictim(false) == Victim::red || CamRec::getVictim(false) == Victim::yellow) {
					consRightOneCol++;
					consRightZeroCol = 0;
					consRightH = 0;
					consRightS = 0;
					consRightU = 0;
				}
				else if (CamRec::getVictim(false) == Victim::green) {
					consRightOneCol = 0;
					consRightZeroCol++;
					consRightH = 0;
					consRightS = 0;
					consRightU = 0;
				}
				else if (CamRec::getVictim(false) == Victim::harmed) {
					consRightOneCol = 0;
					consRightZeroCol = 0;
					consRightH++;
					consRightS = 0;
					consRightU = 0;
				}
				else if (CamRec::getVictim(false) == Victim::stable) {
					consRightOneCol = 0;
					consRightZeroCol = 0;
					consRightH = 0;
					consRightS++;
					consRightU = 0;
				}
				else if (CamRec::getVictim(false) == Victim::unharmed) {
					consRightOneCol = 0;
					consRightZeroCol = 0;
					consRightH = 0;
					consRightS = 0;
					consRightU++;
				}
				else {
					consRightOneCol = 0;
					consRightZeroCol = 0;
					consRightH = 0;
					consRightS = 0;
					consRightU = 0;
				}
			}
			else {
				consRightOneCol = 0;
				consRightZeroCol = 0;
				consRightH = 0;
				consRightS = 0;
				consRightU = 0;
			}

			if (consLeftOneCol >= 2 ||
				consLeftZeroCol >= 2 ||
				consRightOneCol >= 2 ||
				consRightZeroCol >= 2) {

				MapCoordinate victimCoordinate = fusedData.robotState.mapCoordinate;

				switch (fusedData.robotState.heading)
				{
				case AbsoluteDir::north:
					victimCoordinate.x = roundf((fusedData.robotState.position.x - 4.0f) / 30.0f);
					break;
				case AbsoluteDir::east:
					victimCoordinate.y = roundf((fusedData.robotState.position.y + 4.0f) / 30.0f);
					break;
				case AbsoluteDir::south:
					victimCoordinate.x = roundf((fusedData.robotState.position.x + 4.0f) / 30.0f);
					break;
				case AbsoluteDir::west:
					victimCoordinate.y = roundf((fusedData.robotState.position.y - 4.0f) / 30.0f);
					break;
				default:
					break;
				}

				Serial.println("Visual victim!");
				bool leftVictim = false;

				RobotState taskStartState = SmoothDriving::getInformation().startState;
				auto taskUid = SmoothDriving::getInformation().uid;

				bool finishRotateRight = false;
				fusedData = SensorFusion::getFusedData();
				if (fusedData.robotState.wheelSpeeds.left > fusedData.robotState.wheelSpeeds.right) {
					finishRotateRight = true;
				}

				SmoothDriving::stop();

				auto startDisp = millis();
				while (millis() - startDisp < 5500) {
					SensorFusion::sensorFusion();

					PowerLEDs::setBrightness(sinf(millis() / SIALSettings::PowerLEDs::blinkPeriod * M_TWOPI) / 2.0f + 0.5f);
				}

				PowerLEDs::setBrightness(SIALSettings::PowerLEDs::defaultPower);

				if (taskUid == DrivingTaskUID::rotate) {
					float goalAngle = fusedData.robotState.globalHeading;

					if (finishRotateRight) {
						goalAngle = floorf(goalAngle);
					}
					else {
						goalAngle = ceilf(goalAngle);
					}

					SmoothDriving::startNewTask(new SmoothDriving::Rotate(goalAngle - fusedData.robotState.globalHeading, sgn(goalAngle - fusedData.robotState.globalHeading) * 2.0f));
				}

				lastVisDetection = millis();
				lastVisVictim = victimCoordinate;

				consLeftOneCol = 0;
				consLeftZeroCol = 0;
				consLeftH = 0;
				consLeftS = 0;
				consLeftU = 0;
				consRightOneCol = 0;
				consRightZeroCol = 0;
				consRightH = 0;
				consRightS = 0;
				consRightU = 0;
			}
		}

		void handleBumper() {
			const auto retreat = []() {
				SmoothDriving::startNewTask(new SmoothDriving::TaskArray{
					new SmoothDriving::ForceSpeed(-4, -20),
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
				if (consBumper > 3) {
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

					while (millis() - startWait < 100) {
						if (Bumper::leftInterrupt && Bumper::rightInterrupt) {
							Serial.println("Both bumper triggered");
							retreat();
							Bumper::leftInterrupt = false;
							Bumper::rightInterrupt = false;
							return;
						}
					}

					auto t = millis();
					while (!SensorFusion::waitForAllDistSens() && (millis() - t < 500)) {
						DistanceSensors::updateDistSensors();
					}

					fusedData = SensorFusion::getFusedData();

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

			static bool start = true;
			static bool toOtherSide = false;
			static bool returnHome = false;

			static uint8_t consSilver = 0;
			static bool ignoreSilver = false;

			FusedData fusedData = SensorFusion::getFusedData();

			// LOP
			if (!Switch::getState()) {
				SmoothDriving::stop();

				Serial.print("Lack of progress - x: ");
				Serial.print(fusedData.robotState.mapCoordinate.x);
				Serial.print(" y: ");
				Serial.println(fusedData.robotState.mapCoordinate.y);

				while (!Switch::getState()) {
					Gyro::updateValues();
					delay(10);
				}

				NewForcedFusionValues newValues;
				newValues.clearCell = true;
				newValues.newHeading = true;
				newValues.newX = true;
				newValues.newY = true;
				newValues.zeroPitch = true;

				newValues.x = 0.0f;
				newValues.y = 0.0f;

				Gyro::updateValues();
				delay(10);
				newValues.heading = getGlobalHeading(Gyro::getForwardVec());

				SensorFusion::setCorrectedState(newValues);
				// Call twice to restart internal timer
				SensorFusion::sensorFusion(true);
				delay(10);
				SensorFusion::sensorFusion();

				do {
					auto t = millis();
					do {
						SensorFusion::updateSensors();
						SensorFusion::distSensFusion();
						SensorFusion::sensorFusion();
					} while (!SensorFusion::waitForAllDistSens() && (millis() - t < 500));
				} while (!SensorFusion::scanSurrounding());

				SensorFusion::updatePosAndRotFromDist(500);

				bool start = true;
				bool toOtherSide = false;
				bool returnHome = false;
				uint8_t consSilver = 0;
				bool ignoreSilver = false;
			}

			if (start) {
				if (startTime == 0) {
					startTime = millis();
				}

				if (!SensorFusion::waitForAllDistSens()) {
					return;
				}

				if (!SensorFusion::scanSurrounding()) {
					return;
				}

				startNewTask(new TaskArray({ new AlignWalls(), new FollowCell(38) }));

				start = false;
			}
			else {
				auto robotState = fusedData.robotState;

				// Stuck during rotation handling
				if (SensorFusion::consecutiveRotStuck > 60) {
					Serial.println("Stuck during rotation!");

					startNewTask(new RotationUnstuck(), true);
				}

				handleBumper();

				// Tile handling
				if (SmoothDriving::getInformation().drivingStraight &&
					(fusedData.robotState.mapCoordinate != homePosition || returnHome) &&
					fabsf(fusedData.robotState.pitch) < 0.04f) {
					FloorTileColour tileColour = ColorSensor::detectTileColour(fusedData.colorSensData.lux);

					if (tileColour == FloorTileColour::black && toOtherSide) {
						Serial.println("reached other side");

						SmoothDriving::startNewTask(new TaskArray{
							new FollowCell(38),
							new Rotate(M_PI, 3.0f)});

						while (!SmoothDriving::getInformation().finished) {
							SensorFusion::updateSensors();
							SensorFusion::distSensFusion();
							SensorFusion::sensorFusion();
							SmoothDriving::updateSpeeds();
							handleBumper();
						}

						Dispenser::dispenseRight(1);

						SmoothDriving::startNewTask(new TaskArray{
							new AlignWalls(),
							new FollowCell(38),
							new AlignWalls() });

						while (!SmoothDriving::getInformation().finished) {
							SensorFusion::updateSensors();
							SensorFusion::distSensFusion();
							SensorFusion::sensorFusion();
							SmoothDriving::updateSpeeds();
							handleBumper();
						}

						ignoreSilver = false;
						returnHome = true;

						return;
					}
					else if (tileColour == FloorTileColour::black && returnHome) {
						while (true);
					}

					if (tileColour == FloorTileColour::silver && !ignoreSilver) {
						consSilver++;
					}
					else {
						consSilver = 0;
					}
				}
				else {
					consSilver = 0;
				}

				if (consSilver >= 4) {
					Serial.println("Silver");
					ignoreSilver = true;

					SmoothDriving::stop();

					while (true) {
						SensorFusion::updateSensors();
						SensorFusion::distSensFusion();
						SensorFusion::sensorFusion();

						if (!SensorFusion::waitForAllDistSens()) {
							continue;
						}

						if (SensorFusion::scanSurrounding()) {
							break;
						}
					}

					fusedData = SensorFusion::getFusedData();

					const AbsoluteDir absLeft = makeAbsolute(RelativeDir::left, fusedData.robotState.heading);
					const AbsoluteDir absRight = makeAbsolute(RelativeDir::right, fusedData.robotState.heading);

					bool left = false;
					bool right = false;

					// Check for walls
					if (fusedData.gridCell.cellConnections & (uint8_t)absLeft) {
						left = true;
					}

					if (fusedData.gridCell.cellConnections & (uint8_t)absRight) {
						right = true;
					}

					// Maybe improve in case of fault detections
					int16_t dist = 0;
					fusedData = SensorFusion::getFusedData();

					switch (fusedData.robotState.heading)
					{
					case AbsoluteDir::north:
						dist = floorf(fusedData.robotState.position.x / 30.0f) * 30.0f - fusedData.robotState.position.x;
						break;
					case AbsoluteDir::east:
						dist = fusedData.robotState.position.y - ceilf(fusedData.robotState.position.y / 30.0f) * 30.0f;
						break;
					case AbsoluteDir::south:
						dist = fusedData.robotState.position.x - ceilf(fusedData.robotState.position.x / 30.0f) * 30.0f;
						break;
					case AbsoluteDir::west:
						dist = floorf(fusedData.robotState.position.y / 30.0f) * 30.0f - fusedData.robotState.position.y;
						break;
					default:
						break;
					}

					dist = Min(dist - 3, 8);

					if (left) {
						SmoothDriving::startNewTask(new TaskArray{ new FollowWall(dist, 30), new Rotate(M_PI_2, 2.0f) });
					}
					else {
						SmoothDriving::startNewTask(new TaskArray{ new FollowWall(dist, 30), new Rotate(-M_PI_2, -2.0f) });
					}

					while (!SmoothDriving::getInformation().finished) {
						SensorFusion::updateSensors();
						SensorFusion::distSensFusion();
						SensorFusion::sensorFusion();
						SmoothDriving::updateSpeeds();
					}

					while (true) {
						CamRec::loop();

						if (CamRec::dataReady) {
							if (left) {
								if (CamRec::getVictim(false) == Victim::blue) {
									SmoothDriving::startNewTask(new FollowCell(38));
									break;
								}
							}
							else {
								if (CamRec::getVictim(true) == Victim::blue) {
									SmoothDriving::startNewTask(new FollowCell(38));
									break;
								}
							}
						}
					}

					delay(1000);

					while (!SmoothDriving::getInformation().finished) {
						SensorFusion::updateSensors();
						SensorFusion::distSensFusion();
						SensorFusion::sensorFusion();
						SmoothDriving::updateSpeeds();
						handleBumper();
					}

					if (left) {
						SmoothDriving::startNewTask(new SmoothDriving::TaskArray{ new AlignWalls(),
							new Rotate(-M_PI_2, -2.0f),
							new AlignWalls(),
							new FollowCell(38),
							new AlignWalls(),
							new Rotate(-M_PI_2, -2.0f),
							new AlignWalls() });
					}
					else {
						SmoothDriving::startNewTask(new SmoothDriving::TaskArray{ new AlignWalls(),
							new Rotate(M_PI_2, 2.0f),
							new AlignWalls(),
							new FollowCell(38),
							new AlignWalls(),
							new Rotate(M_PI_2, 2.0f),
							new AlignWalls() });
					}

					while (!SmoothDriving::getInformation().finished) {
						SensorFusion::updateSensors();
						SensorFusion::distSensFusion();
						SensorFusion::sensorFusion();
						SmoothDriving::updateSpeeds();
						handleBumper();
					}

					delay(1000);

					while (true) {
						DistanceSensors::updateDistSensors();
						fusedData = SensorFusion::getFusedData();

						if ((fusedData.distSensorState.frontLeft == DistSensorStatus::ok && fusedData.distances.frontLeft > 220) ||
							(fusedData.distSensorState.frontRight == DistSensorStatus::ok && fusedData.distances.frontRight > 220)) {
							break;
						}
					}

					Serial.println("Good to go");

					delay(1000);

					// Free to drive to other side
					if (left) {
						SmoothDriving::startNewTask(new SmoothDriving::TaskArray{ new FollowCell(38),
							new Rotate(M_PI_2, 2.0f),
							new AlignWalls(),
							new FollowCell(38),
							new AlignWalls() });
					}
					else {
						SmoothDriving::startNewTask(new SmoothDriving::TaskArray{ new FollowCell(38),
							new Rotate(-M_PI_2, -2.0f),
							new AlignWalls(),
							new FollowCell(38),
							new AlignWalls() });
					}

					toOtherSide = true;

					while (!SmoothDriving::getInformation().finished) {
						SensorFusion::updateSensors();
						SensorFusion::distSensFusion();
						SensorFusion::sensorFusion();
						SmoothDriving::updateSpeeds();
						handleBumper();
					}
				}

				handleHeatVictim(fusedData);
				handleCamVictim(fusedData);

				if (SmoothDriving::getInformation().finished) {
					if (!SensorFusion::waitForAllDistSens()) {
						return;
					}

					if (SensorFusion::scanSurrounding()) {
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
							NewForcedFusionValues correctedState;
							correctedState.clearCell = true;
							SensorFusion::setCorrectedState(correctedState);

							return;
						}
					}
				}
			}
		}
	}
}