#include "../header/SmoothDriving.h"
#include "../header/SensorFusion.h"
#include "../header/MotorControl.h"
#include "../SIALSettings.h"

namespace SIAL {
	namespace SmoothDriving {
		namespace {
			ITask* _currentTask = nullptr;
		}

		void setNewTask(ITask* task, bool forceOverride) {
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

		ITask::ITask(DrivingTaskInformation information) : _information(information) {}

		DrivingTaskInformation ITask::getInformation()
		{
			return _information;
		}

		Stop::Stop() : ITask(DrivingTaskInformation(DrivingTaskUID::stop)) {}

		void Stop::startTask(RobotState startState) {

		}

		WheelSpeeds Stop::updateSpeeds(float dt) {
			_information.finished = true;
			return WheelSpeeds{ 0, 0 };
		}

		ForceSpeed::ForceSpeed(int32_t dist, int16_t speed) : ITask(DrivingTaskInformation(DrivingTaskUID::forceSpeed, true)), _dist(dist), _speed(speed) {}

		void ForceSpeed::startTask(RobotState startState) {
			_startPos = startState.position;
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

		Rotate::Rotate(float angle, float maxAngularVel) : ITask(DrivingTaskInformation(DrivingTaskUID::rotate)), _maxAngularVel(maxAngularVel), _angle(angle) {}

		void Rotate::startTask(RobotState startState)
		{
			_startAngle = startState.globalHeading;
			_accelerate = true;

			_totalTime = _angle / _maxAngularVel * 2.0f;
		}

		FollowWall::FollowWall(int32_t dist, int16_t speed) : ITask(DrivingTaskInformation(DrivingTaskUID::followWall, true)), _dist(dist), _speed(speed), _pid(SIALSettings::Controller::GoToAngle::pidSettings) {}

		void FollowWall::startTask(RobotState startState) {
			_startPos = startState.position;
		}

		WheelSpeeds FollowWall::updateSpeeds(float dt) {
			const auto fusedData = SensorFusion::getFusedData();
			const auto robotState = fusedData.robotState;

			const Vec2f currentPosition = (Vec2f)(robotState.position);

			float drivenDistance;

			if (robotState.heading == AbsoluteDir::north || robotState.heading == AbsoluteDir::south) {
				drivenDistance = fabsf(currentPosition.x - _startPos.x);
			}
			else {
				drivenDistance = fabsf(currentPosition.y - _startPos.y);
			}

			bool stopWithWall = false;
			if (_speed > 0) {
				if (fusedData.distSensorState.frontRight == DistSensorStatus::ok &&
					(fusedData.distances.frontRight / 10.0f - 15.0f + SIALSettings::Mechanics::distSensFrontBackDist / 2.0f + drivenDistance) * 0.8f < _dist) {
					if (fusedData.distSensorState.frontLeft == DistSensorStatus::ok &&
						(fusedData.distances.frontLeft / 10.0f - 15.0f + SIALSettings::Mechanics::distSensFrontBackDist / 2.0f + drivenDistance) * 0.8f < _dist) {
						stopWithWall = true;
					}
				}
			}

			if (drivenDistance >= fabsf(_dist) * 0.95 && !stopWithWall)
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
				else {
					Serial.println("Should never happen... (invalid distance sensor fusion)");
				}

				float goalAngle = -atanf((15.0f - combinedDistToLeftWall) / SIALSettings::Controller::GoToAngle::aheadDistL * sgn(_speed));
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

		// Update speeds for both wheels
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
			if (fabsf(rotatedAngle) >= fabsf(_angle))
			{
				_information.finished = true;

				return WheelSpeeds{ 0, 0 };
			}

			// Accelerate / deccelerate
			if (_accelerate)
			{
				// w(t) = w_max * 2 * t / t_ges => a(t) = w_max * t^2 / t_ges => t(a) = sqrt(a * t_ges / w_max); w(a) = w_max * 2 * sqrt(a * t_ges / w_max) / t_ges = sqrt(4 * a * w_max / t_ges)
				desAngularVel = sqrtf(4.0f * fabsf(rotatedAngle * _maxAngularVel) / _totalTime) * sgn(_maxAngularVel);

				if (fabsf(rotatedAngle) >= fabsf(_angle) / 2.0f)
				{
					_accelerate = false;
				}
			}
			else
			{
				desAngularVel = _maxAngularVel - sqrtf(4.0f * fabsf((rotatedAngle - _angle / 2.0f) * _maxAngularVel) / _totalTime) * sgn(_maxAngularVel);
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
	}
}