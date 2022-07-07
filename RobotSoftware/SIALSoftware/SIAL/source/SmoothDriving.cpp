#include "../header/SmoothDriving.h"
#include "../header/SensorFusion.h"
#include "../header/MotorControl.h"
#include "../SIALSettings.h"

namespace SIAL {
	namespace SmoothDriving {
		ITask* _currentTask = nullptr;

		void startNewTask(ITask* task, bool forceOverride) {
			if (_currentTask == nullptr || forceOverride || _currentTask->getInformation().finished) {
				delete _currentTask;

				_currentTask = task;
				_currentTask->startTask(SensorFusion::getFusedData().robotState);
			}
			else {
				delete task;
			}
		}

		void updateSpeeds() {
			static bool first = true;
			static uint32_t lastTime = 0;
			uint32_t t = millis();
			float dt = 0.0f;

			if (first) {
				first = false;
				lastTime = t;
			}
			else {
				dt = (t - lastTime) / 1000.0f;
				lastTime = t;
			}

			MotorControl::setSpeeds(_currentTask->updateSpeeds(dt));
		}

		DrivingTaskInformation getInformation() {
			return _currentTask->getInformation();
		}

		void stop() {
			delete _currentTask;
			_currentTask = new Stop();
			_currentTask->startTask(RobotState());
			updateSpeeds();
		}

		void bumper(bool left, bool right) {
			const auto uid = getInformation().uid;
			if ((uid != DrivingTaskUID::ramp) &&
				(uid != DrivingTaskUID::stairs) &&
				(uid != DrivingTaskUID::rotate)) {
				stop();
			}
		}

		ITask::ITask(DrivingTaskInformation information) : _information(information) {}

		DrivingTaskInformation ITask::getInformation()
		{
			return _information;
		}

		Stop::Stop() : ITask(DrivingTaskInformation(DrivingTaskUID::stop)) {}

		void Stop::startTask(RobotState startState) {
			_information.startState = startState;
		}

		WheelSpeeds Stop::updateSpeeds(float dt) {
			_information.finished = true;
			return WheelSpeeds{ 0, 0 };
		}

		ForceSpeed::ForceSpeed(int32_t dist, int16_t speed) : ITask(DrivingTaskInformation(DrivingTaskUID::forceSpeed, true)), _dist(dist), _speed(speed) {}

		void ForceSpeed::startTask(RobotState startState) {
			_startPos = startState.position;
			_information.startState = startState;
		}

		WheelSpeeds ForceSpeed::updateSpeeds(float dt) {
			if (_information.finished) {
				return WheelSpeeds{ 0, 0 };
			}

			if ((SensorFusion::getFusedData().robotState.position - _startPos).length() > fabsf(_dist)) {
				_information.finished = true;
				return WheelSpeeds{ 0, 0 };
			}

			return WheelSpeeds{ _speed, _speed };
		}

		Rotate::Rotate(float angle, float maxAngularVel, bool snapOrientation) : ITask(DrivingTaskInformation(DrivingTaskUID::rotate)), _maxAngularVel(maxAngularVel), _angle(angle), _snapOrientation(snapOrientation), _consFinished(0) {}

		void Rotate::startTask(RobotState startState)
		{
			_startAngle = startState.globalHeading;

			if (_snapOrientation) {
				float end = _startAngle + _angle;
				end = roundf(end / M_PI_2) * M_PI_2;
				_angle = end - _startAngle;
			}

			_accelerate = true;

			_totalTime = _angle / _maxAngularVel * 2.0f;

			_information.startState = startState;
		}

		WheelSpeeds Rotate::updateSpeeds(float dt)
		{
			float rotatedAngle;			// Rotated angle since start
			float desAngularVel;		// Desired angular velocity
			WheelSpeeds output;			// Output

			if (_information.finished) {
				return WheelSpeeds{ 0, 0 };
			}

			const auto tempRobotState = SensorFusion::getFusedData().robotState;

			// Calculate rotated angle
			rotatedAngle = tempRobotState.globalHeading - _startAngle;

			// Check if I am there
			if (fabsf(rotatedAngle) >= fabsf(_angle) - 3.0f * DEG_TO_RAD)
			{
				_consFinished++;
			}
			else {
				_consFinished = 0;
			}

			if (_consFinished >= 2) {
				_information.finished = true;

				return WheelSpeeds{ 0, 0 };
			}

			// Accelerate / deccelerate
			if (_accelerate)
			{
				// w(t) = w_max * t / (t_ges / 2) => a(t) = w_max * t^2 / t_ges => t(a) = sqrt(a * t_ges / w_max)
				float t = sqrtf(fabsf(rotatedAngle * _totalTime / _maxAngularVel));
				desAngularVel = _maxAngularVel * 2.0f * t / _totalTime;

				if (fabsf(rotatedAngle) >= fabsf(_angle) / 2.0f)
				{
					_accelerate = false;
				}
			}
			else
			{
				float t = _totalTime / 2.0f - sqrtf(fabsf((_angle - rotatedAngle) * _totalTime / _maxAngularVel));
				desAngularVel = _maxAngularVel - _maxAngularVel * 2.0f * t / _totalTime;
			}

			// Compute wheel speeds -- w = (v_r - v_l) / wheelDistance; v_l = -v_r; => v_l = -w * wheelDistance / 2; v_r = w * wheelDistance / 2
			output = WheelSpeeds{ -SIALSettings::Mechanics::wheelDistance * desAngularVel / 2.0f, SIALSettings::Mechanics::wheelDistance * desAngularVel / 2.0f };

			// Correct speed if it is too low
			if (output.right < SIALSettings::MotorControl::minSpeed && output.right > -SIALSettings::MotorControl::minSpeed)
			{
				output.right = SIALSettings::MotorControl::minSpeed * sgn(_angle);
				output.left = -output.right;
			}

			return output;
		}

		RotationUnstuck::RotationUnstuck() : ITask(DrivingTaskInformation(DrivingTaskUID::rotationUnstuck)), _retreatStage(true), _retreatStart(UINT32_MAX) {}

		void RotationUnstuck::startTask(RobotState startState)
		{
			_information.startState = startState;
			if (startState.wheelSpeeds.left > startState.wheelSpeeds.right) {
				_angle = floorf(startState.globalHeading / M_PI_2) * M_PI_2 - startState.globalHeading;
			}
			else {
				_angle = ceilf(startState.globalHeading / M_PI_2) * M_PI_2 - startState.globalHeading;
			}
			_startAngle = startState.globalHeading;
		}

		WheelSpeeds RotationUnstuck::updateSpeeds(float dt)
		{
			float rotatedAngle;			// Rotated angle since start
			float desAngularVel;		// Desired angular velocity
			WheelSpeeds output;			// Output

			if (_information.finished) {
				return WheelSpeeds{ 0, 0 };
			}

			const auto tempRobotState = SensorFusion::getFusedData().robotState;

			// Calculate rotated angle
			rotatedAngle = tempRobotState.globalHeading - _startAngle;

			// Check if I am there
			if (fabsf(rotatedAngle) >= fabsf(_angle) - 5.0f * DEG_TO_RAD && !_retreatStage)
			{
				_consFinished++;
			}
			else {
				_consFinished = 0;
			}

			if (_consFinished >= 2) {
				_information.finished = true;

				return WheelSpeeds{ 0, 0 };
			}

			if (_retreatStage) {
				if (_retreatStart == UINT32_MAX) {
					_retreatStart = millis();
				}
				else if (millis() - _retreatStart > 400) {
					_retreatStage = false;
				}
				
				desAngularVel = -2.0f * sgn(_angle);
			}
			else {
				desAngularVel = 4.0f * sgn(_angle);
			}

			// Compute wheel speeds -- w = (v_r - v_l) / wheelDistance; v_l = -v_r; => v_l = -w * wheelDistance / 2; v_r = w * wheelDistance / 2
			output = WheelSpeeds{ -SIALSettings::Mechanics::wheelDistance * desAngularVel / 2.0f, SIALSettings::Mechanics::wheelDistance * desAngularVel / 2.0f };

			// Correct speed if it is too low
			if (output.right < SIALSettings::MotorControl::minSpeed && output.right > -SIALSettings::MotorControl::minSpeed)
			{
				output.right = SIALSettings::MotorControl::minSpeed * sgn(_angle);
				output.left = -output.right;
			}

			return output;
		}

		FollowWall::FollowWall(int32_t dist, int16_t speed) : ITask(DrivingTaskInformation(DrivingTaskUID::followWall, true)), _dist(dist), _speed(speed), _pid(SIALSettings::Controller::GoToAngle::pidSettings) {}

		void FollowWall::startTask(RobotState startState) {
			_startPos = startState.position;
			_information.startState = startState;

			Serial.print("Start FollowWall at: ");
			Serial.print(_information.startState.position.x);
			Serial.print(", ");
			Serial.print(_information.startState.position.y);
			Serial.print(" with dist: ");
			Serial.println(_dist);
		}

		WheelSpeeds FollowWall::updateSpeeds(float dt) {
			const auto fusedData = SensorFusion::getFusedData();
			const auto robotState = fusedData.robotState;

			const Vec2f currentPosition = robotState.position;

			float drivenDistance;

			if (robotState.heading == AbsoluteDir::north || robotState.heading == AbsoluteDir::south) {
				drivenDistance = fabsf(currentPosition.x - _startPos.x);
			}
			else {
				drivenDistance = fabsf(currentPosition.y - _startPos.y);
			}

			if (drivenDistance >= fabsf(_dist))
			{
				Serial.println("Stopped FollowWall -> drivenDistance");
				_information.finished = true;
			}

			if (fusedData.distSensorState.frontRight == DistSensorStatus::underflow && fusedData.distSensorState.frontLeft == DistSensorStatus::underflow) {
				Serial.println("Stopped FollowWall -> underflow");
				_information.finished = true;
			}

			if (_information.finished) {
				return WheelSpeeds(0, 0);
			}

			float errorAngle = 0.0f;

			if (fusedData.fusedDistSens.distSensAngleTrust > 0.01f) {
				float combinedDistToLeftWall = -1.0f;

				if (fusedData.fusedDistSens.distToWalls.l > 0.0f && fusedData.fusedDistSens.distToWalls.r > 0.0f) {
					combinedDistToLeftWall = (fusedData.fusedDistSens.distToWalls.l + 30.0f - fusedData.fusedDistSens.distToWalls.r) / 2.0;
				}
				else if (fusedData.fusedDistSens.distToWalls.l > 0.0f) {
					combinedDistToLeftWall = fusedData.fusedDistSens.distToWalls.l;
				}
				else if (fusedData.fusedDistSens.distToWalls.r > 0.0f) {
					combinedDistToLeftWall = 30.0f - fusedData.fusedDistSens.distToWalls.r;
				}

				float goalAngle;
				if (combinedDistToLeftWall > 0.0f) {
					goalAngle = -atanf((15.0f - combinedDistToLeftWall) / SIALSettings::Controller::GoToAngle::aheadDistL * sgn(_speed));
				}
				else {
					goalAngle = 0;
				}
				errorAngle = fitAngleToInterval(goalAngle - fitAngleToInterval(fusedData.fusedDistSens.distSensAngle * 4.0f) / 4.0f);
			}
			else {
				errorAngle = -fitAngleToInterval(fusedData.robotState.globalHeading * 4.0f) / 4.0f;
			}

			// point forward steering http://faculty.salina.k-state.edu/tim/robot_prog/MobileBot/Steering/pointFwd.html
			errorAngle = _pid.process(0.0f, -errorAngle, dt);
			float correctedAngularVel = fabsf(_speed) * errorAngle;
			float correctedForwardVel = fmaxf(fabsf(_speed * cosf(fmaxf(fminf(errorAngle, M_PI_2), -M_PI_2))), SIALSettings::MotorControl::minSpeed) * sgn(_speed);

			// Compute wheel speeds - v = (v_r + v_l) / 2; w = (v_r - v_l) / wheelDistance => v_l = v - w * wheelDistance / 2; v_r = v + w * wheelDistance / 2
			WheelSpeeds output = WheelSpeeds{ correctedForwardVel - SIALSettings::Mechanics::wheelDistance * correctedAngularVel / 2.0f, correctedForwardVel + SIALSettings::Mechanics::wheelDistance * correctedAngularVel / 2.0f };

			// Correct speed if it is too low
			if (output.left < SIALSettings::MotorControl::minSpeed && output.left > -SIALSettings::MotorControl::minSpeed)
			{
				output.left = SIALSettings::MotorControl::minSpeed * sgn(_dist);
			}

			if (output.right < SIALSettings::MotorControl::minSpeed && output.right > -SIALSettings::MotorControl::minSpeed)
			{
				output.right = SIALSettings::MotorControl::minSpeed * sgn(_dist);
			}

			return output;
		}

		FollowCell::FollowCell(int16_t speed) : ITask(DrivingTaskInformation(DrivingTaskUID::followCell, true)), _wallTask(0, speed), _speed(speed) {}

		void FollowCell::startTask(RobotState startState) {
			int32_t dist = 0;

			switch (startState.heading)
			{
			case AbsoluteDir::north:
			{
				dist = (int32_t)((startState.mapCoordinate.x + sgn(_speed)) * 30.0f - startState.position.x);
				break;
			}
			case AbsoluteDir::east:
			{
				dist = (int32_t)(startState.position.y - (startState.mapCoordinate.y - sgn(_speed)) * 30.0f);
				break;
			}
			case AbsoluteDir::south:
			{
				dist = (int32_t)(startState.position.x - (startState.mapCoordinate.x - sgn(_speed)) * 30.0f);
				break;
			}
			case AbsoluteDir::west:
			{
				dist = (int32_t)((startState.mapCoordinate.y + sgn(_speed)) * 30.0f - startState.position.y);
				break;
			}
			default:
				break;
			}

			if (sgn(dist) != sgn(_speed)) {
				dist = 0;
			}

			_wallTask = FollowWall(dist, _speed);
			_wallTask.startTask(startState);
			_information = _wallTask.getInformation();
			_information.uid = DrivingTaskUID::followCell;
		}

		WheelSpeeds FollowCell::updateSpeeds(float dt) {
			auto wheelSpeeds = _wallTask.updateSpeeds(dt);
			_information = _wallTask.getInformation();
			_information.uid = DrivingTaskUID::followCell;
			return wheelSpeeds;
		}

		Ramp::Ramp(uint8_t speed, bool up) : ITask(DrivingTaskInformation(DrivingTaskUID::ramp, true)), _speed(speed), _emaPitch(NAN), _up(up), _consecutiveEnd(0), _rampEnd(false), _pid(SIALSettings::Controller::GoToAngle::pidSettings) {}

		void Ramp::startTask(RobotState startState) {
			_information.startState = startState;
		}

		WheelSpeeds Ramp::updateSpeeds(float dt) {
			const auto fusedData = SensorFusion::getFusedData();
			const auto robotState = fusedData.robotState;

			if (!_rampEnd) {
				if (std::isnan(_emaPitch)) {
					_emaPitch = robotState.pitch;
				}
				else {
					_emaPitch = robotState.pitch * 0.6f + _emaPitch * 0.4f;
				}

				if (_emaPitch * (_up ? 1.0f : -1.0f) < 0.1f) {
					_consecutiveEnd++;
				}
				else {
					_consecutiveEnd = 0;
				}

				if (_consecutiveEnd >= 15) {
					_rampEnd = true;
					_rampEndPos = robotState.position;
				}
			}
			else {
				const Vec2f currentPosition = robotState.position;

				float drivenDistance;

				if (robotState.heading == AbsoluteDir::north || robotState.heading == AbsoluteDir::south) {
					drivenDistance = fabsf(currentPosition.x - _rampEndPos.x);
				}
				else {
					drivenDistance = fabsf(currentPosition.y - _rampEndPos.y);
				}

				if (drivenDistance >= fabsf(10) - fabsf(_speed) * 0.06f)
				{
					_information.finished = true;
				}

				if (fusedData.distSensorState.frontRight == DistSensorStatus::underflow ||
					(fusedData.distSensorState.frontRight == DistSensorStatus::ok &&
						fusedData.distances.frontRight < (30.0f - SIALSettings::Mechanics::distSensFrontBackDist) / 2 * 10 * 0.9f)) {
					_information.finished = true;
				}

				if (fusedData.distSensorState.frontLeft == DistSensorStatus::underflow ||
					(fusedData.distSensorState.frontLeft == DistSensorStatus::ok &&
						fusedData.distances.frontLeft < (30.0f - SIALSettings::Mechanics::distSensFrontBackDist) / 2 * 10 * 0.9f)) {
					_information.finished = true;
				}
			}

			if (_information.finished) {
				return WheelSpeeds(0, 0);
			}

			float errorAngle = 0.0f;

			if (fusedData.fusedDistSens.distSensAngleTrust > 0.01f) {
				float combinedDistToLeftWall = -1.0f;

				if (fusedData.fusedDistSens.distToWalls.l > 0.0f && fusedData.fusedDistSens.distToWalls.r > 0.0f) {
					combinedDistToLeftWall = (fusedData.fusedDistSens.distToWalls.l + 30.0f - fusedData.fusedDistSens.distToWalls.r) / 2.0;
				}
				else if (fusedData.fusedDistSens.distToWalls.l > 0.0f) {
					combinedDistToLeftWall = fusedData.fusedDistSens.distToWalls.l;
				}
				else if (fusedData.fusedDistSens.distToWalls.r > 0.0f) {
					combinedDistToLeftWall = 30.0f - fusedData.fusedDistSens.distToWalls.r;
				}

				float goalAngle;
				if (combinedDistToLeftWall > 0.0f) {
					goalAngle = -atanf((15.0f - combinedDistToLeftWall) / SIALSettings::Controller::GoToAngle::aheadDistL);
				}
				else {
					goalAngle = 0;
				}
				errorAngle = fitAngleToInterval(goalAngle - fitAngleToInterval(fusedData.fusedDistSens.distSensAngle * 4.0f) / 4.0f);
			}
			else {
				errorAngle = -fitAngleToInterval(fusedData.robotState.globalHeading * 4.0f) / 4.0f;
			}

			// point forward steering http://faculty.salina.k-state.edu/tim/robot_prog/MobileBot/Steering/pointFwd.html
			errorAngle = _pid.process(0.0f, -errorAngle, dt);
			float correctedAngularVel = _speed * errorAngle;
			float correctedForwardVel = fmaxf(fabsf(_speed * cosf(fmaxf(fminf(errorAngle, M_PI_2), -M_PI_2))), SIALSettings::MotorControl::minSpeed);

			// Compute wheel speeds - v = (v_r + v_l) / 2; w = (v_r - v_l) / wheelDistance => v_l = v - w * wheelDistance / 2; v_r = v + w * wheelDistance / 2
			WheelSpeeds output = WheelSpeeds{ correctedForwardVel - SIALSettings::Mechanics::wheelDistance * correctedAngularVel / 2.0f, correctedForwardVel + SIALSettings::Mechanics::wheelDistance * correctedAngularVel / 2.0f };

			// Correct speed if it is too low
			if (output.left < SIALSettings::MotorControl::minSpeed && output.left > -SIALSettings::MotorControl::minSpeed)
			{
				output.left = SIALSettings::MotorControl::minSpeed;
			}

			if (output.right < SIALSettings::MotorControl::minSpeed && output.right > -SIALSettings::MotorControl::minSpeed)
			{
				output.right = SIALSettings::MotorControl::minSpeed;
			}

			return output;
		}

		Stairs::Stairs(uint8_t speed) : ITask(DrivingTaskInformation(DrivingTaskUID::stairs, true)), _speed(speed), _emaPitch(NAN), _pid(SIALSettings::Controller::GoToAngle::pidSettings), _up(true), _consEnd(0), _stairEnd(false) {}

		void Stairs::startTask(RobotState startState) {
			_information.startState = startState;
		}

		WheelSpeeds Stairs::updateSpeeds(float dt) {
			const auto fusedData = SensorFusion::getFusedData();
			const auto robotState = fusedData.robotState;

			if (std::isnan(_emaPitch)) {
				_emaPitch = robotState.pitch;
			}
			else {
				_emaPitch = robotState.pitch * 0.65f + _emaPitch * 0.35f;
			}

			if (_stairEnd && !_up && _emaPitch < -0.05f) {
				_stairEnd = false;
				_consEnd = 0;
			}

			if (!_stairEnd) {
				if (_up && _emaPitch < -0.08f) {
					_up = false;
				}
				else if (!_up && _emaPitch > -0.08f) {
					_consEnd++;
				}
				else {
					_consEnd = 0;
				}

				if (_consEnd >= 15) {
					_stairEnd = true;
					_stairEndPos = robotState.position;
				}
			}
			else {
				const Vec2f currentPosition = robotState.position;

				float drivenDistance;

				if (robotState.heading == AbsoluteDir::north || robotState.heading == AbsoluteDir::south) {
					drivenDistance = fabsf(currentPosition.x - _stairEndPos.x);
				}
				else {
					drivenDistance = fabsf(currentPosition.y - _stairEndPos.y);
				}

				if (drivenDistance >= fabsf(5) - fabsf(_speed) * 0.06f)
				{
					_information.finished = true;
				}

				if (fusedData.distSensorState.frontRight == DistSensorStatus::underflow ||
					(fusedData.distSensorState.frontRight == DistSensorStatus::ok &&
						fusedData.distances.frontRight < (30.0f - SIALSettings::Mechanics::distSensFrontBackDist) / 2 * 10 * 0.9f)) {
					_information.finished = true;
				}

				if (fusedData.distSensorState.frontLeft == DistSensorStatus::underflow ||
					(fusedData.distSensorState.frontLeft == DistSensorStatus::ok &&
						fusedData.distances.frontLeft < (30.0f - SIALSettings::Mechanics::distSensFrontBackDist) / 2 * 10 * 0.9f)) {
					_information.finished = true;
				}
			}

			if (_information.finished) {
				return WheelSpeeds(0, 0);
			}

			float errorAngle = 0.0f;

			if (fusedData.fusedDistSens.distSensAngleTrust > 0.01f) {
				float combinedDistToLeftWall = -1.0f;

				if (fusedData.fusedDistSens.distToWalls.l > 0.0f && fusedData.fusedDistSens.distToWalls.r > 0.0f) {
					combinedDistToLeftWall = (fusedData.fusedDistSens.distToWalls.l + 30.0f - fusedData.fusedDistSens.distToWalls.r) / 2.0;
				}
				else if (fusedData.fusedDistSens.distToWalls.l > 0.0f) {
					combinedDistToLeftWall = fusedData.fusedDistSens.distToWalls.l;
				}
				else if (fusedData.fusedDistSens.distToWalls.r > 0.0f) {
					combinedDistToLeftWall = 30.0f - fusedData.fusedDistSens.distToWalls.r;
				}

				float goalAngle;
				if (combinedDistToLeftWall > 0.0f) {
					goalAngle = -atanf((15.0f - combinedDistToLeftWall) / SIALSettings::Controller::GoToAngle::aheadDistL);
				}
				else {
					goalAngle = 0;
				}
				errorAngle = fitAngleToInterval(goalAngle - fitAngleToInterval(fusedData.fusedDistSens.distSensAngle * 4.0f) / 4.0f);
			}
			else {
				errorAngle = -fitAngleToInterval(fusedData.robotState.globalHeading * 4.0f) / 4.0f;
			}

			// point forward steering http://faculty.salina.k-state.edu/tim/robot_prog/MobileBot/Steering/pointFwd.html
			errorAngle = _pid.process(0.0f, -errorAngle, dt);
			float correctedAngularVel = _speed * errorAngle;
			float correctedForwardVel = fmaxf(fabsf(_speed * cosf(fmaxf(fminf(errorAngle, M_PI_2), -M_PI_2))), SIALSettings::MotorControl::minSpeed);

			// Compute wheel speeds - v = (v_r + v_l) / 2; w = (v_r - v_l) / wheelDistance => v_l = v - w * wheelDistance / 2; v_r = v + w * wheelDistance / 2
			WheelSpeeds output = WheelSpeeds{ correctedForwardVel - SIALSettings::Mechanics::wheelDistance * correctedAngularVel / 2.0f, correctedForwardVel + SIALSettings::Mechanics::wheelDistance * correctedAngularVel / 2.0f };

			// Correct speed if it is too low
			if (output.left < SIALSettings::MotorControl::minSpeed && output.left > -SIALSettings::MotorControl::minSpeed)
			{
				output.left = SIALSettings::MotorControl::minSpeed;
			}

			if (output.right < SIALSettings::MotorControl::minSpeed && output.right > -SIALSettings::MotorControl::minSpeed)
			{
				output.right = SIALSettings::MotorControl::minSpeed;
			}

			return output;
		}

		AlignWalls::AlignWalls() : ITask(DrivingTaskInformation(DrivingTaskUID::alignWalls)), _absAvgAngle(0.0f), _first(true), _time(0.0f), _consOk(0) {}

		void AlignWalls::startTask(RobotState startState) {
			_information.startState = startState;
		}

		WheelSpeeds AlignWalls::updateSpeeds(float dt)
		{
			static WheelSpeeds output;

			_time += dt;

			if (_consOk >= 5 && !_information.finished) {
				SensorFusion::forceAnglePosReset = true;
				_information.finished = true;
			}
			else if (_time >= 1.0f) {
				_information.finished = true;
			}

			if (_information.finished)
			{
				return WheelSpeeds{ 0,0 };
			}

			float angle = SensorFusion::getAngleRelToWall();

			if (_first) {
				_absAvgAngle = fabsf(angle);
				_first = false;
			}
			else {
				_absAvgAngle = fabsf(angle) * 0.8f + _absAvgAngle * 0.2f;
			}

			if ((_absAvgAngle < 0.02)) {
				_consOk++;
				return WheelSpeeds{ 0,0 };
			}
			else {
				_consOk = 0;
				if (std::isnan(angle)) {
					// If there is no wall -> stop faster
					_time += dt * 3.0f;
					return WheelSpeeds{ 0,0 };
				}
			}

			if (angle > 0.0f) {
				output.left = SIALSettings::MotorControl::minSpeed;
				output.right = -SIALSettings::MotorControl::minSpeed;
			}
			else {
				output.left = -SIALSettings::MotorControl::minSpeed;
				output.right = SIALSettings::MotorControl::minSpeed;
			}

			return output;
		}

		TaskArray::TaskArray(std::initializer_list<SmoothDriving::ITask*> tasks) : ITask(DrivingTaskInformation(DrivingTaskUID::invalid)) {
			_num = tasks.size();
			_tasks = new ITask * [_num];

			uint8_t i = 0;
			for (ITask* task : tasks) {
				_tasks[i++] = task;
			}
		}

		TaskArray::~TaskArray() {
			for (uint8_t i = 0; i < _num; i++) {
				delete _tasks[i];
			}
			delete[] _tasks;
		}

		void TaskArray::startTask(RobotState startState) {
			_i = 0;
			_tasks[_i]->startTask(startState);
			_information = _tasks[_i]->getInformation();
			_information.finished = false;
		}

		WheelSpeeds TaskArray::updateSpeeds(float dt) {
			_information = _tasks[_i]->getInformation();

			if (_information.finished) {
				if (_i + 1 >= _num) {
					_information.finished = true;
				}
				else {
					_i++;
					_tasks[_i]->startTask(SensorFusion::getFusedData().robotState);
					_information = _tasks[_i]->getInformation();
					_information.finished = false;
				}
			}

			return _tasks[_i]->updateSpeeds(dt);
		}
	}
}