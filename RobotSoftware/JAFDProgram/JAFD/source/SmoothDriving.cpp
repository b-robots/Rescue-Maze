/*
This part of the Library is responsible for driving smoothly.
*/

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include <algorithm>

#include "../header/SmoothDriving.h"
#include "../header/MotorControl.h"
#include "../header/SensorFusion.h"
#include "../../JAFDSettings.h"
#include "../header/Math.h"
#include "../header/PIDController.h"
#include "../header/DistanceSensors.h"
#include "../header/SensorFusion.h"
#include "cmath"

namespace JAFD
{
	namespace SmoothDriving
	{
		float debug1 = 0.0;
		float debug2 = 0.0;
		float debug3 = 0.0;
		float debug4 = 0.0;

		using namespace JAFDSettings::Controller;

		namespace
		{
			// Copy of current task
			union _TaskCopies
			{
				Accelerate accelerate;
				DriveStraight straight;
				Stop stop;
				Rotate rotate;
				ForceSpeed forceSpeed;
				TaskArray taskArray;
				AlignWalls alignWalls;
				FollowWall followWall;

				_TaskCopies() : stop() {};
				~_TaskCopies() {}
			} _taskCopies;

			ITask* _currentTask = &_taskCopies.stop;	// Current task

			PIDController _forwardVelPID(JAFDSettings::Controller::SmoothDriving::forwardVelPidSettings);	// PID controller for forward velocity
			PIDController _angularVelPID(JAFDSettings::Controller::SmoothDriving::angularVelPidSettings);	// PID controller for angular velocity

			volatile bool _stopped = false;		// Is current task stopped?
		}

		ITask::ITask() : _finished(false), _endState() {}

		bool ITask::isFinished()
		{
			return _finished;
		}

		RobotState ITask::getEndState()
		{
			return _endState;
		}

		// Accelerate class - begin 

		Accelerate::Accelerate(int16_t endSpeeds, float distance) : ITask(), _endSpeeds(endSpeeds), _distance(distance), _targetDir(1.0f, 0.0f) {}

		ReturnCode Accelerate::startTask(RobotState startState)
		{
			_finished = false;
			_targetDir = Vec2f(cosf(startState.globalHeading), sinf(startState.globalHeading));
			_startPos = (Vec2f)(startState.position);
			_startSpeeds = startState.forwardVel;

			if (!((_startSpeeds >= 0 && _endSpeeds >= 0 && _distance >= 0) || (_startSpeeds <= 0 && _endSpeeds <= 0 && _distance <= 0)) || _endSpeeds == _startSpeeds) return ReturnCode::error;

			_totalTime = 2 * _distance / static_cast<float>(_endSpeeds + _startSpeeds);

			_endState.wheelSpeeds = FloatWheelSpeeds{ _endSpeeds, _endSpeeds };
			_endState.forwardVel = static_cast<float>(_endSpeeds);
			_endState.position = startState.position + (Vec3f)(_targetDir * _distance);
			_endState.angularVel = Vec3f(0.0f, 0.0f, 0.0f);
			_endState.globalHeading = startState.globalHeading;

			return ReturnCode::ok;
		}

		// Update speeds for both wheels
		WheelSpeeds Accelerate::updateSpeeds(const uint8_t freq)
		{
			Vec2f currentPosition;			// Current position of robot
			float currentHeading;			// Current heading of robot;
			Vec2f posRelToStart;			// Position relative to start
			float drivenDistance;			// Distance to startpoint (with correct sign for direction)
			float radiant;					// Needed as temporary value for acceleration/decceleration
			float calculatedTime;			// Calculated time based on driven distance
			float desiredSpeed;				// Desired linear velocity
			float desAngularVel;			// Desired angular velocity
			float correctedForwardVel;		// Corrected forward velocity
			float correctedAngularVel;		// Corrected angular velocity
			WheelSpeeds output;				// Speed output for both wheels

			const auto tempRobotState = SensorFusion::getFusedData().robotState;

			currentPosition = (Vec2f)(tempRobotState.position);
			currentHeading = tempRobotState.globalHeading;

			// Calculate driven distance
			posRelToStart = currentPosition - _startPos;
			drivenDistance = std::max(posRelToStart.x * _targetDir.x + posRelToStart.y * _targetDir.y, 0.0f);

			// Check if I am there
			if (drivenDistance >= fabsf(_distance))
			{
				_finished = true;

				if (_endSpeeds == 0)
				{
					_forwardVelPID.reset();
					_angularVelPID.reset();

					return WheelSpeeds{ 0, 0 };
				}
			}

			drivenDistance *= sgn(_distance);

			// If finished, drive with end speeds
			if (!_finished)
			{
				// Accelerate / deccelerate - v = v_1 + (t / t_ges) * (v_2 - v_1); s = _errorIntegral(v * dt) = ((v_2 - v_1) * t^2) / (2 * t_ges) + v_1 * t => radiant = t_ges * v_1^2 - 2 * s * (v_1 + v_2); t = t_ges * (v_1 - sqrt(radiant)) / (v_2 - v_1); t_ges = s * 2 / (v_2 - v_1)
				radiant = static_cast<float>(_startSpeeds) * static_cast<float>(_startSpeeds) + 2.0f * drivenDistance * static_cast<float>(_endSpeeds - _startSpeeds) / _totalTime;
				if (radiant < 0.0f) radiant = 0.0f;
				calculatedTime = _totalTime * (static_cast<float>(_startSpeeds) - sqrtf(radiant) * sgn(_startSpeeds + _endSpeeds)) / static_cast<float>(_startSpeeds - _endSpeeds);
				if (calculatedTime < 0.0f) calculatedTime = 0.0f;
				desiredSpeed = static_cast<float>(_startSpeeds) + (calculatedTime / _totalTime) * static_cast<float>(_endSpeeds - _startSpeeds);
			}
			else
			{
				desiredSpeed = _endSpeeds;
			}

			// TODO
			// GoToAngle Algorithm - funktionier nicht
			Vec3f goToVec = _endState.position - tempRobotState.position;

			float angleDamping = std::max(GoToAngle::angleDampingBegin - fabsf(fabsf(drivenDistance) - fabsf(_distance)), 0.0f) / GoToAngle::angleDampingBegin;

			float errorAngle = fitAngleToInterval(getGlobalHeading(goToVec) - tempRobotState.globalHeading);
			errorAngle *= 1.0f - angleDamping;

			desAngularVel = desiredSpeed / GoToAngle::aheadDistL * sinf(-errorAngle);
			desiredSpeed = desiredSpeed * cosf(-errorAngle);

			//// A variation of pure pursuits controller where the goal point is a lookahead distance on the path away (not a lookahead distance from the robot).
			//// Furthermore, the lookahead distance is dynamically adapted to the speed
			//// Calculate goal point
			//lookAheadDistance = JAFDSettings::Controller::PurePursuit::lookAheadGain * desiredSpeed;

			//if (lookAheadDistance < JAFDSettings::Controller::PurePursuit::minLookAheadDist && lookAheadDistance > -JAFDSettings::Controller::PurePursuit::minLookAheadDist) lookAheadDistance = JAFDSettings::Controller::PurePursuit::minLookAheadDist * sgn(desiredSpeed);

			//goalPointGlobal = _startPos + _targetDir * (posRelToStart.length() * sgn(_distance) + lookAheadDistance);

			//// Transform goal point to robot coordinates
			//goalPointRobot.x = (goalPointGlobal.x - currentPosition.x)  * cosf(currentHeading) + (goalPointGlobal.y - currentPosition.y) * sinf(currentHeading);
			//goalPointRobot.y = -(goalPointGlobal.x - currentPosition.x) * sinf(currentHeading) + (goalPointGlobal.y - currentPosition.y) * cosf(currentHeading);

			//// Calculate curvature and angular velocity
			//desCurvature = 2.0f * goalPointRobot.y / (goalPointRobot.x * goalPointRobot.x + goalPointRobot.y * goalPointRobot.y);
			//
			//if (desCurvature > JAFDSettings::Controller::PurePursuit::maxCurvature) desCurvature = JAFDSettings::Controller::PurePursuit::maxCurvature;
			//else if (desCurvature < -JAFDSettings::Controller::PurePursuit::maxCurvature) desCurvature = -JAFDSettings::Controller::PurePursuit::maxCurvature;

			//desAngularVel = desiredSpeed * desCurvature;

			// Kind of PID - controller
			correctedForwardVel = desiredSpeed * PID::nonePIDPart + _forwardVelPID.process(desiredSpeed, tempRobotState.forwardVel, 1.0f / freq);
			correctedAngularVel = desAngularVel * PID::nonePIDPart + _angularVelPID.process(desAngularVel, tempRobotState.angularVel.x, 1.0f / freq);

			// Compute wheel speeds - v = (v_r + v_l) / 2; w = (v_r - v_l) / wheelDistance => v_l = v - w * wheelDistance / 2; v_r = v + w * wheelDistance / 2
			output = WheelSpeeds{ correctedForwardVel - JAFDSettings::Mechanics::wheelDistance * correctedAngularVel / 2.0f, correctedForwardVel + JAFDSettings::Mechanics::wheelDistance * correctedAngularVel / 2.0f };

			// Correct speed if it is too low 
			if (output.left < JAFDSettings::MotorControl::minSpeed && output.left > -JAFDSettings::MotorControl::minSpeed)
			{
				_forwardVelPID.reset();
				_angularVelPID.reset();

				output.left = JAFDSettings::MotorControl::minSpeed * sgn(_distance);
			}

			if (output.right < JAFDSettings::MotorControl::minSpeed && output.right > -JAFDSettings::MotorControl::minSpeed)
			{
				_forwardVelPID.reset();
				_angularVelPID.reset();

				output.right = JAFDSettings::MotorControl::minSpeed * sgn(_distance);
			}

			return output;
		}

		// Accelerate class - end

		// DriveStraight class - begin

		DriveStraight::DriveStraight(float distance) : ITask(), _distance(distance), _targetDir(1.0f, 0.0f) {}

		ReturnCode DriveStraight::startTask(RobotState startState)
		{
			_finished = false;
			_targetDir = Vec2f(cosf(startState.globalHeading), sinf(startState.globalHeading));
			_startPos = (Vec2f)(startState.position);
			_speeds = startState.forwardVel;

			if (sgn(_speeds) != sgn(_distance)) return ReturnCode::error;

			if (_speeds <= 0)
			{
				_targetDir *= -1;
			}

			_endState.wheelSpeeds = FloatWheelSpeeds{ _speeds, _speeds };
			_endState.forwardVel = static_cast<float>(_speeds);
			_endState.position = startState.position + (Vec3f)(_targetDir * fabsf(_distance));
			_endState.angularVel = Vec3f(0.0f, 0.0f, 0.0f);
			_endState.globalHeading = startState.globalHeading;

			return ReturnCode::ok;
		}

		// Update speeds for both wheels
		WheelSpeeds DriveStraight::updateSpeeds(const uint8_t freq)
		{
			Vec2f currentPosition;			// Current position of robot
			float currentHeading;			// Current heading of robot;
			Vec2f posRelToStart;			// Position relative to start
			float absDrivenDist;			// Absolute distance to startpoint
			float radiant;					// Needed as temporary value for acceleration/decceleration
			float calculatedTime;			// Calculated time based on driven distance
			float desAngularVel;			// Desired angular velocity
			float desiredSpeed;				// Speed calculated by "GoToAngle" algorithm
			WheelSpeeds output;				// Speed output for both wheels
			float correctedForwardVel;		// Corrected forward velocity
			float correctedAngularVel;		// Corrected angular velocity

			const auto tempRobotState = SensorFusion::getFusedData().robotState;

			currentPosition = (Vec2f)(tempRobotState.position);
			currentHeading = tempRobotState.globalHeading;

			// Calculate driven distance
			posRelToStart = currentPosition - _startPos;
			absDrivenDist = posRelToStart.length();

			// Check if I am there
			if (absDrivenDist >= fabsf(_distance))
			{
				_finished = true;
			}

			// GoToAngle Algorithm
			Vec3f goToVec = _endState.position - (Vec3f)currentPosition;

			float angleDamping = std::max(GoToAngle::angleDampingBegin - fabsf(absDrivenDist - fabsf(_distance)), 0.0f) / GoToAngle::angleDampingBegin;

			float errorAngle = fitAngleToInterval(getGlobalHeading(goToVec) - tempRobotState.globalHeading);
			errorAngle *= 1.0f - angleDamping;

			desAngularVel = _speeds / GoToAngle::aheadDistL * sinf(errorAngle);
			desiredSpeed = _speeds * cosf(errorAngle);

			//// A variation of pure pursuits controller where the goal point is a lookahead distance on the path away (not a lookahead distance from the robot).
			//// Furthermore, the lookahead distance is dynamically adapted to the speed
			//// Calculate goal point
			//lookAheadDistance = JAFDSettings::Controller::PurePursuit::lookAheadGain * _speeds;
			//lookAheadDistance = fabsf(lookAheadDistance);

			//if (lookAheadDistance < JAFDSettings::Controller::PurePursuit::minLookAheadDist) lookAheadDistance = JAFDSettings::Controller::PurePursuit::minLookAheadDist;

			//goalPointGlobal = _startPos + _targetDir * (absDrivenDist + lookAheadDistance);

			//// Transform goal point to robot coordinates
			//goalPointRobot.x = (goalPointGlobal.x - currentPosition.x)  * cosf(currentHeading) + (goalPointGlobal.y - currentPosition.y) * sinf(currentHeading);
			//goalPointRobot.y = -(goalPointGlobal.x - currentPosition.x) * sinf(currentHeading) + (goalPointGlobal.y - currentPosition.y) * cosf(currentHeading);

			//// Calculate curvature and angular velocity
			//desCurvature = 2.0f * goalPointRobot.y / (goalPointRobot.x * goalPointRobot.x + goalPointRobot.y * goalPointRobot.y);

			//if (desCurvature > JAFDSettings::Controller::PurePursuit::maxCurvature) desCurvature = JAFDSettings::Controller::PurePursuit::maxCurvature;
			//else if (desCurvature < -JAFDSettings::Controller::PurePursuit::maxCurvature) desCurvature = -JAFDSettings::Controller::PurePursuit::maxCurvature;

			//desAngularVel = _speeds * desCurvature;

			// Kind of PID - controller
			correctedForwardVel = desiredSpeed * PID::nonePIDPart + _forwardVelPID.process(_speeds, tempRobotState.forwardVel, 1.0f / freq);
			correctedAngularVel = desAngularVel * PID::nonePIDPart + _angularVelPID.process(desAngularVel, tempRobotState.angularVel.x, 1.0f / freq);

			// Compute wheel speeds - v = (v_r + v_l) / 2; w = (v_r - v_l) / wheelDistance => v_l = v - w * wheelDistance / 2; v_r = v + w * wheelDistance / 2
			output = WheelSpeeds{ correctedForwardVel - JAFDSettings::Mechanics::wheelDistance * correctedAngularVel / 2.0f, correctedForwardVel + JAFDSettings::Mechanics::wheelDistance * correctedAngularVel / 2.0f };

			// Correct speed if it is too low 
			if (output.left < JAFDSettings::MotorControl::minSpeed && output.left > -JAFDSettings::MotorControl::minSpeed)
			{
				_forwardVelPID.reset();
				_angularVelPID.reset();

				output.left = JAFDSettings::MotorControl::minSpeed * sgn(_distance);
			}

			if (output.right < JAFDSettings::MotorControl::minSpeed && output.right > -JAFDSettings::MotorControl::minSpeed)
			{
				_forwardVelPID.reset();
				_angularVelPID.reset();

				output.right = JAFDSettings::MotorControl::minSpeed * sgn(_distance);
			}

			return output;
		}

		// DriveStraight class - end

		// Stop class - begin

		ReturnCode Stop::startTask(RobotState startState)
		{
			_finished = false;
			_endState.wheelSpeeds = FloatWheelSpeeds{ 0.0f, 0.0f };
			_endState.forwardVel = static_cast<float>(0.0f);
			_endState.position = startState.position;
			_endState.angularVel = Vec3f(0.0f, 0.0f, 0.0f);
			_endState.globalHeading = startState.globalHeading;

			_forwardVelPID.reset();
			_angularVelPID.reset();

			return ReturnCode::ok;
		}

		WheelSpeeds Stop::updateSpeeds(const uint8_t freq)
		{
			_finished = true;
			return WheelSpeeds{ 0, 0 };
		}

		// Stop class - end

		// Rotate class - begin

		Rotate::Rotate(float maxAngularVel, float angle) : ITask(), _maxAngularVel(maxAngularVel), _angle(angle / 180.0f * M_PI), _accelerate(true) {}

		ReturnCode Rotate::startTask(RobotState startState)
		{
			_finished = false;
			_startAngle = startState.globalHeading;
			_accelerate = true;

			if (abs(startState.wheelSpeeds.left) > JAFDSettings::MotorControl::minSpeed || abs(startState.wheelSpeeds.right) > JAFDSettings::MotorControl::minSpeed) return ReturnCode::error;

			_totalTime = _angle / _maxAngularVel * 2.0f;

			if (_totalTime < 0.0f) return ReturnCode::error;

			_endState.wheelSpeeds = FloatWheelSpeeds{ 0.0f, 0.0f };
			_endState.forwardVel = static_cast<float>(0.0f);
			_endState.position = startState.position;
			_endState.angularVel = Vec3f(0.0f, 0.0f, 0.0f);
			_endState.globalHeading = startState.globalHeading + _angle;

			_forwardVelPID.reset();
			_angularVelPID.reset();

			return ReturnCode::ok;
		}

		// Update speeds for both wheels
		WheelSpeeds Rotate::updateSpeeds(const uint8_t freq)
		{
			float rotatedAngle;			// Rotated angle since start
			float desAngularVel;		// Desired angular velocity
			float correctedAngularVel;	// By PID Controller corrected angular velocity
			WheelSpeeds output;			// Output

			const auto tempRobotState = SensorFusion::getFusedData().robotState;

			// Calculate rotated angle
			rotatedAngle = tempRobotState.globalHeading - _startAngle;

			// Check if I am there
			if (fabsf(rotatedAngle) >= fabsf(_angle))
			{
				_forwardVelPID.reset();
				_angularVelPID.reset();

				_finished = true;

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

			// Kind of PID - controller
			correctedAngularVel = desAngularVel * 0.8f + _angularVelPID.process(desAngularVel, tempRobotState.angularVel.x, 1.0f / freq);

			// Compute wheel speeds -- w = (v_r - v_l) / wheelDistance; v_l = -v_r; => v_l = -w * wheelDistance / 2; v_r = w * wheelDistance / 2
			output = WheelSpeeds{ -JAFDSettings::Mechanics::wheelDistance * correctedAngularVel / 2.0f, JAFDSettings::Mechanics::wheelDistance * correctedAngularVel / 2.0f };

			// Correct speed if it is too low
			if (output.right < JAFDSettings::MotorControl::minSpeed && output.right > -JAFDSettings::MotorControl::minSpeed)
			{
				_angularVelPID.reset();
				output.right = JAFDSettings::MotorControl::minSpeed * sgn(_angle);
				output.left = -output.right;
			}

			return output;
		}

		// Rotate class - end

		// ForceSpeed class - begin

		ForceSpeed::ForceSpeed(int16_t speed, float distance) : ITask(), _speeds(speed), _distance(distance) {}

		ReturnCode ForceSpeed::startTask(RobotState startState)
		{
			_finished = false;
			_startPos = (Vec2f)(startState.position);

			float startAngle = startState.globalHeading;
			startAngle = fitAngleToInterval(startAngle);

			if (startAngle > -M_PI_4 && startAngle < M_PI_4) {
				startAngle = 0.0f;
			}
			else if (startAngle > M_PI_4 && startAngle < M_3PI_4) {
				startAngle = M_PI_2;
			}
			else if (startAngle < -M_PI_4 && startAngle > -M_3PI_4) {
				startAngle = -M_PI_2;
			}
			else
			{
				startAngle = M_PI;
			}

			_endState.wheelSpeeds = FloatWheelSpeeds{ _speeds, _speeds };
			_endState.forwardVel = static_cast<float>(_speeds);
			_endState.position = startState.position + (Vec3f)(Vec2f(cosf(startAngle), sinf(startAngle)) * _distance);
			_endState.angularVel = Vec3f(0.0f, 0.0f, 0.0f);
			_endState.globalHeading = startState.globalHeading;

			return ReturnCode::ok;
		}

		// Update speeds for both wheels
		WheelSpeeds ForceSpeed::updateSpeeds(const uint8_t freq)
		{
			if (((Vec2f)SensorFusion::getFusedData().robotState.position - _startPos).length() > fabsf(_distance)) {
				_finished = true;
			}

			if (_finished) {
				return WheelSpeeds{ 0, 0 };
			}

			return WheelSpeeds{ _speeds, _speeds};
		}

		// ForceSpeed class - end

		// AlignWalls class - begin
		AlignWalls::AlignWalls() : ITask(), _avgAngle(0.0f), _first(true), _cnt(0), _waitForPosReset(false) {
		}

		ReturnCode AlignWalls::startTask(RobotState startState)
		{
			_finished = false;
			_endState = startState;
			return ReturnCode::ok;
		}

		WheelSpeeds AlignWalls::updateSpeeds(const uint8_t freq)
		{
			static WheelSpeeds output;

			_cnt++;

			float angle = SensorFusion::getAngleRelToWall();

			if (_cnt / (float)freq > 2.0f || (fabsf(_avgAngle) < 0.04 && !_first)) {
				if (!_waitForPosReset) {
					SensorFusion::forceAnglePosReset = true;
					_waitForPosReset = true;
				}
			}
			else {
				if (std::isnan(angle)) {
					return WheelSpeeds{ 0,0 };
				}

				if (_first) {
					_avgAngle = angle;
					_first = false;
				}
				else {
					_avgAngle = angle * 0.8 + _avgAngle * 0.2;
				}
			}

			if (_waitForPosReset && !SensorFusion::forceAnglePosReset) {
				_finished = true;
				_waitForPosReset = false;
			}

			if (_finished || _waitForPosReset)
			{
				return WheelSpeeds{ 0,0 };
			}

			if (_avgAngle > 0.0f) {
				output.left = JAFDSettings::MotorControl::minSpeed;
				output.right = -JAFDSettings::MotorControl::minSpeed;
			}
			else {
				output.left = -JAFDSettings::MotorControl::minSpeed;
				output.right = JAFDSettings::MotorControl::minSpeed;
			}

			return output;
		}

		// AlignWalls class - end

		// FollowWall class - begin

		FollowWall::FollowWall(int16_t speed, float distance) : ITask(), _speeds(speed), _distance(distance), _pid(JAFDSettings::Controller::GoToAngle::pidSettings) {}

		ReturnCode FollowWall::startTask(RobotState startState) {
			_finished = false;
			_startPos = (Vec2f)(startState.position);
			_startAngle = startState.globalHeading;
			_startAngle = fitAngleToInterval(_startAngle);

			if (_startAngle > -M_PI_4 && _startAngle < M_PI_4) {
				_startAngle = 0.0f;
			}
			else if (_startAngle > M_PI_4 && _startAngle < M_3PI_4) {
				_startAngle = M_PI_2;
			}
			else if (_startAngle < -M_PI_4 && _startAngle > -M_3PI_4) {
				_startAngle = -M_PI_2;
			}
			else
			{
				_startAngle = M_PI;
			}

			if (sgn(_speeds) != sgn(_distance)) return ReturnCode::error;

			_endState.wheelSpeeds = FloatWheelSpeeds{ _speeds, _speeds };
			_endState.forwardVel = static_cast<float>(_speeds);
			_endState.position = startState.position + (Vec3f)(Vec2f(cosf(_startAngle), sinf(_startAngle)) * _distance);
			_endState.angularVel = Vec3f(0.0f, 0.0f, 0.0f);
			_endState.globalHeading = makeRotationCoherent(startState.globalHeading, _startAngle);

			_pid.reset();

			return ReturnCode::ok;
		}

		WheelSpeeds FollowWall::updateSpeeds(const uint8_t freq) {
			const auto tempFusedData = SensorFusion::getFusedData();
			const auto tempRobotState = tempFusedData.robotState;

			const Vec2f currentPosition = (Vec2f)(tempRobotState.position);

			float drivenDistance;

			if (fabsf(_startAngle - 0.0f) < 0.1f || fabsf(_startAngle - M_PI) < 0.1f) {
				drivenDistance = fabsf(currentPosition.x - _startPos.x);
			}
			else {
				drivenDistance = fabsf(currentPosition.y - _startPos.y);
			}

			bool stopWithWall = false;
			if (_speeds > 0) {
				if (tempFusedData.distSensorState.frontRight == DistSensorStatus::ok &&
					(tempFusedData.distances.frontRight / 10.0f - 15.0f + JAFDSettings::Mechanics::distSensFrontBackDist / 2.0f + drivenDistance) * 0.8f < _distance) {
					if (tempFusedData.distSensorState.frontLeft == DistSensorStatus::ok &&
						(tempFusedData.distances.frontLeft / 10.0f - 15.0f + JAFDSettings::Mechanics::distSensFrontBackDist / 2.0f + drivenDistance) * 0.8f < _distance) {
						stopWithWall = true;
					}
				}
			}

			if (drivenDistance >= fabsf(_distance) * 0.95 && !stopWithWall)
			{
				_finished = true;
			}

			if (tempFusedData.distSensorState.frontRight == DistSensorStatus::ok &&
				tempFusedData.distances.frontRight < (JAFDSettings::Field::cellWidth - JAFDSettings::Mechanics::distSensFrontBackDist) / 2 * 10 * 0.9f) {
				_finished = true;
			}

			if (tempFusedData.distSensorState.frontLeft == DistSensorStatus::ok &&
				tempFusedData.distances.frontLeft < (JAFDSettings::Field::cellWidth - JAFDSettings::Mechanics::distSensFrontBackDist) / 2 * 10 * 0.9f) {
				_finished = true;
			}

			if (_finished) {
				return WheelSpeeds(0, 0);
			}

			struct {
				bool lf = false;
				bool lb = false;
				bool rf = false;
				bool rb = false;
				bool fl = false;
				bool fr = false;
			} usableData;

			struct {
				int lf = 0;
				int lb = 0;
				int rf = 0;
				int rb = 0;
				int fl = 0;
				int fr = 0;
			} distances;

			if ((tempFusedData.distSensorState.leftFront == DistSensorStatus::ok &&
				tempFusedData.distances.leftFront < 300 - JAFDSettings::Mechanics::distSensLeftRightDist * 10) ||
				tempFusedData.distSensorState.leftFront == DistSensorStatus::underflow) {
				usableData.lf = true;
				distances.lf = tempFusedData.distSensorState.leftFront == DistSensorStatus::underflow ? DistanceSensors::VL6180::minDist : tempFusedData.distances.leftFront;
			}

			if ((tempFusedData.distSensorState.leftBack == DistSensorStatus::ok &&
				tempFusedData.distances.leftBack < 300 - JAFDSettings::Mechanics::distSensLeftRightDist * 10) ||
				tempFusedData.distSensorState.leftBack == DistSensorStatus::underflow) {
				usableData.lb = true;
				distances.lb = tempFusedData.distSensorState.leftBack == DistSensorStatus::underflow ? DistanceSensors::VL6180::minDist : tempFusedData.distances.leftBack;
			}

			if ((tempFusedData.distSensorState.rightFront == DistSensorStatus::ok &&
				tempFusedData.distances.rightFront < 300 - JAFDSettings::Mechanics::distSensLeftRightDist * 10) ||
				tempFusedData.distSensorState.rightFront == DistSensorStatus::underflow) {
				usableData.rf = true;
				distances.rf = tempFusedData.distSensorState.rightFront == DistSensorStatus::underflow ? DistanceSensors::VL6180::minDist : tempFusedData.distances.rightFront;
			}

			if ((tempFusedData.distSensorState.rightBack == DistSensorStatus::ok &&
				tempFusedData.distances.rightBack < 300 - JAFDSettings::Mechanics::distSensLeftRightDist * 10) ||
				tempFusedData.distSensorState.rightBack == DistSensorStatus::underflow) {
				usableData.rb = true;
				distances.rb = tempFusedData.distSensorState.rightBack == DistSensorStatus::underflow ? DistanceSensors::VL6180::minDist : tempFusedData.distances.rightBack;
			}

			if (tempFusedData.distSensorState.frontLeft == DistSensorStatus::ok &&
				tempFusedData.distances.frontLeft < 300) {
				usableData.fl = true;
				distances.fl = tempFusedData.distances.frontLeft;
			}

			if (tempFusedData.distSensorState.frontRight == DistSensorStatus::ok &&
				tempFusedData.distances.frontRight < 300) {
				usableData.fr = true;
				distances.fr = tempFusedData.distances.frontRight;
			}

			float errorAngle = 0.0f;
			float angle = tempFusedData.robotState.globalHeading;
			float goalAngle = _startAngle;

			if (usableData.lf && usableData.lb && usableData.rb && usableData.rf) {
				float angleL = 0.0f;
				float distToWallL = 0.0f;
				float angleR = 0.0f;
				float distToWallR = 0.0f;

				calcAngleWallOffsetFromTwoDistances(&angleL, &distToWallL, distances.lf, distances.lb, JAFDSettings::Mechanics::distSensLRSpacing, JAFDSettings::Mechanics::distSensLeftRightDist);
				calcAngleWallOffsetFromTwoDistances(&angleR, &distToWallR, distances.rf, distances.rb, JAFDSettings::Mechanics::distSensLRSpacing, JAFDSettings::Mechanics::distSensLeftRightDist);
				angleL *= -1.0f;

				angle = (angleL + angleR) / 2.0;
				float combinedDistToLeftWall = (distToWallL + (JAFDSettings::Field::cellWidth - distToWallR)) / 2.0;

				goalAngle = -atanf((JAFDSettings::Field::cellWidth / 2.0f - combinedDistToLeftWall) / GoToAngle::aheadDistL * sgn(_speeds));
			}
			else if (usableData.lf && usableData.lb) {
				angle = 0.0f;
				float distToWall = 0.0f;

				calcAngleWallOffsetFromTwoDistances(&angle, &distToWall, distances.lf, distances.lb, JAFDSettings::Mechanics::distSensLRSpacing, JAFDSettings::Mechanics::distSensLeftRightDist);
				angle *= -1.0f;

				goalAngle = -atanf((JAFDSettings::Field::cellWidth / 2.0f - 1.0f - distToWall) / GoToAngle::aheadDistL * sgn(_speeds));
			}
			else if (usableData.rf && usableData.rb) {
				angle = 0.0f;
				float distToWall = 0.0f;

				calcAngleWallOffsetFromTwoDistances(&angle, &distToWall, distances.rf, distances.rb, JAFDSettings::Mechanics::distSensLRSpacing, JAFDSettings::Mechanics::distSensLeftRightDist);

				goalAngle = atanf((JAFDSettings::Field::cellWidth / 2.0f - 1.0f - distToWall) / GoToAngle::aheadDistL * sgn(_speeds));
			}

			errorAngle = fitAngleToInterval(goalAngle - angle);

			// point forward steering http://faculty.salina.k-state.edu/tim/robot_prog/MobileBot/Steering/pointFwd.html
			errorAngle = fmaxf(fminf(errorAngle, M_PI_2), -M_PI_2);
			errorAngle = _pid.process(0.0f, -errorAngle, 1.0f / freq);
			float correctedAngularVel = fabsf(_speeds) * errorAngle;
			float correctedForwardVel = fmaxf(fabsf(_speeds * cosf(fmaxf(fminf(errorAngle, M_PI_2), -M_PI_2))), JAFDSettings::MotorControl::minSpeed) * sgn(_speeds);

			// Compute wheel speeds - v = (v_r + v_l) / 2; w = (v_r - v_l) / wheelDistance => v_l = v - w * wheelDistance / 2; v_r = v + w * wheelDistance / 2
			WheelSpeeds output = WheelSpeeds{ correctedForwardVel - JAFDSettings::Mechanics::wheelDistance * correctedAngularVel / 2.0f, correctedForwardVel + JAFDSettings::Mechanics::wheelDistance * correctedAngularVel / 2.0f };

			// Correct speed if it is too low 
			if (output.left < JAFDSettings::MotorControl::minSpeed && output.left > -JAFDSettings::MotorControl::minSpeed)
			{
				output.left = JAFDSettings::MotorControl::minSpeed * sgn(_distance);
			}

			if (output.right < JAFDSettings::MotorControl::minSpeed && output.right > -JAFDSettings::MotorControl::minSpeed)
			{
				output.right = JAFDSettings::MotorControl::minSpeed * sgn(_distance);
			}

			return output;
		}

		// FollowWall class - end

		// TaskArray class - begin

		TaskArray::TaskArray(const TaskArray& taskArray) : ITask(), _numTasks(taskArray._numTasks), _currentTaskNum(taskArray._numTasks - 1)
		{
			_endState = taskArray._endState;

			for (uint8_t i = 0; i < _numTasks; i++)
			{
				_taskTypes[i] = taskArray._taskTypes[i];

				switch (_taskTypes[i])
				{
				case _TaskType::accelerate:
					_taskArray[i] = new (&(_taskCopies[i].accelerate)) Accelerate(taskArray._taskCopies[i].accelerate);
					break;
				case _TaskType::straight:
					_taskArray[i] = new (&(_taskCopies[i].straight)) DriveStraight(taskArray._taskCopies[i].straight);
					break;
				case _TaskType::stop:
					_taskArray[i] = new (&(_taskCopies[i].stop)) Stop(taskArray._taskCopies[i].stop);
					break;
				case _TaskType::rotate:
					_taskArray[i] = new (&(_taskCopies[i].rotate)) Rotate(taskArray._taskCopies[i].rotate);
					break;
				case _TaskType::forceSpeed:
					_taskArray[i] = new (&(_taskCopies[i].forceSpeed)) ForceSpeed(taskArray._taskCopies[i].forceSpeed);
					break;
				case _TaskType::followWall:
					_taskArray[i] = new (&(_taskCopies[i].followWall)) FollowWall(taskArray._taskCopies[i].followWall);
					break;
				case _TaskType::alignWalls:
					_taskArray[i] = new (&(_taskCopies[i].alignWalls)) AlignWalls(taskArray._taskCopies[i].alignWalls);
					break;
				default:
					break;
				}
			}
		}

		TaskArray::TaskArray(const Accelerate& task) : ITask()
		{
			_taskTypes[_numTasks] = _TaskType::accelerate;
			_taskArray[_numTasks] = new(&(_taskCopies[_numTasks].accelerate)) Accelerate(task);
			_currentTaskNum = _numTasks;
			_numTasks++;
		}

		TaskArray::TaskArray(const DriveStraight& task) : ITask()
		{
			_taskTypes[_numTasks] = _TaskType::straight;
			_taskArray[_numTasks] = new(&(_taskCopies[_numTasks].straight)) DriveStraight(task);
			_currentTaskNum = _numTasks;
			_numTasks++;
		}

		TaskArray::TaskArray(const Stop& task) : ITask()
		{
			_taskTypes[_numTasks] = _TaskType::stop;
			_taskArray[_numTasks] = new(&(_taskCopies[_numTasks].stop)) Stop(task);
			_currentTaskNum = _numTasks;
			_numTasks++;
		}

		TaskArray::TaskArray(const Rotate& task) : ITask()
		{
			_taskTypes[_numTasks] = _TaskType::rotate;
			_taskArray[_numTasks] = new(&(_taskCopies[_numTasks].rotate)) Rotate(task);
			_currentTaskNum = _numTasks;
			_numTasks++;
		}

		TaskArray::TaskArray(const ForceSpeed& task) : ITask()
		{
			_taskTypes[_numTasks] = _TaskType::forceSpeed;
			_taskArray[_numTasks] = new(&(_taskCopies[_numTasks].forceSpeed)) ForceSpeed(task);
			_currentTaskNum = _numTasks;
			_numTasks++;
		}

		TaskArray::TaskArray(const AlignWalls& task)
		{
			_taskTypes[_numTasks] = _TaskType::alignWalls;
			_taskArray[_numTasks] = new(&(_taskCopies[_numTasks].alignWalls)) AlignWalls(task);
			_currentTaskNum = _numTasks;
			_numTasks++;
		}

		TaskArray::TaskArray(const FollowWall& task) : ITask()
		{
			_taskTypes[_numTasks] = _TaskType::followWall;
			_taskArray[_numTasks] = new(&(_taskCopies[_numTasks].followWall)) FollowWall(task);
			_currentTaskNum = _numTasks;
			_numTasks++;
		}

		ReturnCode TaskArray::startTask(RobotState startState)
		{
			ReturnCode code = ReturnCode::ok;
			RobotState state = startState;

			for (int16_t i = _numTasks - 1; i >= 0; i--)
			{
				if (_taskArray[i]->startTask(state) != ReturnCode::ok)
				{
					code = ReturnCode::error;
				}

				state = _taskArray[i]->getEndState();
			}

			_endState = state;

			return code;
		}

		WheelSpeeds TaskArray::updateSpeeds(const uint8_t freq)
		{
			WheelSpeeds speeds = _taskArray[_currentTaskNum]->updateSpeeds(freq);

			if (_taskArray[_currentTaskNum]->isFinished())
			{
				if (_currentTaskNum <= 0)
				{
					_finished = true;
					return speeds;
				}

				_currentTaskNum--;

				_taskArray[_currentTaskNum]->startTask(_taskArray[_currentTaskNum + 1]->getEndState());
			}

			return speeds;
		}

		// TaskArray class - end

		// Update speeds for both wheels
		void updateSpeeds(const uint8_t freq)
		{
			if (!_stopped)
			{
				MotorControl::setSpeeds(_currentTask->updateSpeeds(freq));
			}
			else
			{
				_angularVelPID.reset();
				_forwardVelPID.reset();
			}
		}

		// Set new Accelerate task (use last end state to start)
		template<>
		ReturnCode setNewTask<NewStateType::lastEndState>(const Accelerate& newTask, const bool forceOverride)
		{
			static RobotState endState;
			static ReturnCode returnCode;
			static Accelerate temp;

			returnCode = ReturnCode::ok;

			__disable_irq();

			if (_currentTask->isFinished() || forceOverride)
			{
				endState = static_cast<RobotState>(_currentTask->getEndState());

				temp = newTask;
				returnCode = temp.startTask(endState);

				if (returnCode == ReturnCode::ok)
				{
					_currentTask = new (&(_taskCopies.accelerate)) Accelerate(temp);
					_stopped = false;
				}
			}

			__enable_irq();
			return returnCode;
		}

		// Set new Accelerate task (use current state to start)
		template<>
		ReturnCode setNewTask<NewStateType::currentState>(const Accelerate& newTask, const bool forceOverride)
		{
			static ReturnCode returnCode;
			static Accelerate temp;

			returnCode = ReturnCode::ok;

			__disable_irq();

			if (_currentTask->isFinished() || forceOverride)
			{
				temp = newTask;
				returnCode = temp.startTask(SensorFusion::getFusedData().robotState);

				if (returnCode == ReturnCode::ok)
				{
					_currentTask = new (&(_taskCopies.accelerate)) Accelerate(temp);
					_stopped = false;
				}
			}

			__enable_irq();
			return returnCode;
		}

		// Set new Accelerate task (use specified state to start)
		ReturnCode setNewTask(const Accelerate& newTask, RobotState startState, const bool forceOverride)
		{
			static ReturnCode returnCode;
			static Accelerate temp;

			returnCode = ReturnCode::ok;

			__disable_irq();

			if (_currentTask->isFinished() || forceOverride)
			{
				temp = newTask;
				returnCode = temp.startTask(startState);

				if (returnCode == ReturnCode::ok)
				{
					_currentTask = new (&(_taskCopies.accelerate)) Accelerate(temp);
					_stopped = false;
				}
			}

			__enable_irq();
			return returnCode;
		}

		// Set new DriveStraight task (use last end state to start)
		template<>
		ReturnCode setNewTask<NewStateType::lastEndState>(const DriveStraight& newTask, const bool forceOverride)
		{
			static RobotState endState;
			static ReturnCode returnCode;
			static DriveStraight temp;

			returnCode = ReturnCode::ok;

			__disable_irq();

			if (_currentTask->isFinished() || forceOverride)
			{
				endState = static_cast<RobotState>(_currentTask->getEndState());

				temp = newTask;
				returnCode = temp.startTask(endState);

				if (returnCode == ReturnCode::ok)
				{
					_currentTask = new (&(_taskCopies.straight)) DriveStraight(temp);
					_stopped = false;
				}
			}

			__enable_irq();
			return returnCode;
		}

		// Set new DriveStraight task (use current state to start)
		template<>
		ReturnCode setNewTask<NewStateType::currentState>(const DriveStraight& newTask, const bool forceOverride)
		{
			static ReturnCode returnCode;
			static DriveStraight temp;

			returnCode = ReturnCode::ok;

			__disable_irq();

			if (_currentTask->isFinished() || forceOverride)
			{
				temp = newTask;
				returnCode = temp.startTask(SensorFusion::getFusedData().robotState);

				if (returnCode == ReturnCode::ok)
				{
					_currentTask = new (&(_taskCopies.straight)) DriveStraight(temp);
					_stopped = false;
				}
			}

			__enable_irq();
			return returnCode;
		}

		// Set new DriveStraight task (use specified state to start)
		ReturnCode setNewTask(const DriveStraight& newTask, RobotState startState, const bool forceOverride)
		{
			static ReturnCode returnCode;
			static DriveStraight temp;

			returnCode = ReturnCode::ok;

			__disable_irq();

			if (_currentTask->isFinished() || forceOverride)
			{
				temp = newTask;
				returnCode = temp.startTask(startState);

				if (returnCode == ReturnCode::ok)
				{
					_currentTask = new (&(_taskCopies.straight)) DriveStraight(temp);
					_stopped = false;
				}
			}

			__enable_irq();
			return returnCode;
		}

		// Set new Stop task (use last end state to start)
		template<>
		ReturnCode setNewTask<NewStateType::lastEndState>(const Stop& newTask, const bool forceOverride)
		{
			static RobotState endState;
			static ReturnCode returnCode;
			static Stop temp;

			returnCode = ReturnCode::ok;

			__disable_irq();

			if (_currentTask->isFinished() || forceOverride)
			{
				endState = static_cast<RobotState>(_currentTask->getEndState());

				temp = newTask;
				returnCode = temp.startTask(endState);

				if (returnCode == ReturnCode::ok)
				{
					_currentTask = new (&(_taskCopies.stop)) Stop(temp);
					_stopped = false;
				}
			}

			__enable_irq();
			return returnCode;
		}

		// Set new Stop task (use current state to start)
		template<>
		ReturnCode setNewTask<NewStateType::currentState>(const Stop& newTask, const bool forceOverride)
		{
			static ReturnCode returnCode;
			static Stop temp;

			returnCode = ReturnCode::ok;

			__disable_irq();

			if (_currentTask->isFinished() || forceOverride)
			{
				temp = newTask;
				returnCode = temp.startTask(SensorFusion::getFusedData().robotState);

				if (returnCode == ReturnCode::ok)
				{
					_currentTask = new (&(_taskCopies.stop)) Stop(temp);
					_stopped = false;
				}
			}

			__enable_irq();
			return returnCode;
		}

		// Set new Stop task (use specified state to start)
		ReturnCode setNewTask(const Stop& newTask, RobotState startState, const bool forceOverride)
		{
			static ReturnCode returnCode;
			static Stop temp;

			returnCode = ReturnCode::ok;

			__disable_irq();

			if (_currentTask->isFinished() || forceOverride)
			{
				temp = newTask;
				returnCode = temp.startTask(startState);

				if (returnCode == ReturnCode::ok)
				{
					_currentTask = new (&(_taskCopies.stop)) Stop(temp);
					_stopped = false;
				}
			}

			__enable_irq();
			return returnCode;
		}

		// Set new Rotate task (use last end state to start)
		template<>
		ReturnCode setNewTask<NewStateType::lastEndState>(const Rotate& newTask, const bool forceOverride)
		{
			static RobotState endState;
			static ReturnCode returnCode;
			static Rotate temp;

			returnCode = ReturnCode::ok;

			__disable_irq();

			if (_currentTask->isFinished() || forceOverride)
			{
				endState = static_cast<RobotState>(_currentTask->getEndState());

				temp = newTask;
				returnCode = temp.startTask(endState);

				if (returnCode == ReturnCode::ok)
				{
					_currentTask = new (&(_taskCopies.rotate)) Rotate(temp);
					_stopped = false;
				}
			}

			__enable_irq();
			return returnCode;
		}

		// Set new Rotate task (use current state to start)
		template<>
		ReturnCode setNewTask<NewStateType::currentState>(const Rotate& newTask, const bool forceOverride)
		{
			static ReturnCode returnCode;
			static Rotate temp;

			returnCode = ReturnCode::ok;

			__disable_irq();

			if (_currentTask->isFinished() || forceOverride)
			{
				temp = newTask;
				returnCode = temp.startTask(SensorFusion::getFusedData().robotState);

				if (returnCode == ReturnCode::ok)
				{
					_currentTask = new (&(_taskCopies.rotate)) Rotate(temp);
					_stopped = false;
				}
			}

			__enable_irq();
			return returnCode;
		}

		// Set new Rotate task (use specified state to start)
		ReturnCode setNewTask(const Rotate& newTask, RobotState startState, const bool forceOverride)
		{
			static ReturnCode returnCode;
			static Rotate temp;

			returnCode = ReturnCode::ok;

			__disable_irq();

			if (_currentTask->isFinished() || forceOverride)
			{
				temp = newTask;
				returnCode = temp.startTask(startState);

				if (returnCode == ReturnCode::ok)
				{
					_currentTask = new (&(_taskCopies.rotate)) Rotate(temp);
					_stopped = false;
				}
			}

			__enable_irq();
			return returnCode;
		}

		// Set new ForceSpeed task (use last end state to start)
		template<>
		ReturnCode setNewTask<NewStateType::lastEndState>(const ForceSpeed& newTask, const bool forceOverride)
		{
			static RobotState endState;
			static ReturnCode returnCode;
			static ForceSpeed temp;

			returnCode = ReturnCode::ok;

			__disable_irq();

			if (_currentTask->isFinished() || forceOverride)
			{
				endState = static_cast<RobotState>(_currentTask->getEndState());

				temp = newTask;
				returnCode = temp.startTask(endState);

				if (returnCode == ReturnCode::ok)
				{
					_currentTask = new (&(_taskCopies.forceSpeed)) ForceSpeed(temp);
					_stopped = false;
				}
			}

			__enable_irq();
			return returnCode;
		}

		// Set new ForceSpeed task (use current state to start)
		template<>
		ReturnCode setNewTask<NewStateType::currentState>(const ForceSpeed& newTask, const bool forceOverride)
		{
			static ReturnCode returnCode;
			static ForceSpeed temp;

			returnCode = ReturnCode::ok;

			__disable_irq();

			if (_currentTask->isFinished() || forceOverride)
			{
				temp = newTask;
				returnCode = temp.startTask(SensorFusion::getFusedData().robotState);

				if (returnCode == ReturnCode::ok)
				{
					_currentTask = new (&(_taskCopies.forceSpeed)) ForceSpeed(temp);
					_stopped = false;
				}
			}

			__enable_irq();
			return returnCode;
		}

		// Set new ForceSpeed task (use specified state to start)
		ReturnCode setNewTask(const ForceSpeed& newTask, RobotState startState, const bool forceOverride)
		{
			static ReturnCode returnCode;
			static ForceSpeed temp;

			returnCode = ReturnCode::ok;

			__disable_irq();

			if (_currentTask->isFinished() || forceOverride)
			{
				temp = newTask;
				returnCode = temp.startTask(startState);

				if (returnCode == ReturnCode::ok)
				{
					_currentTask = new (&(_taskCopies.forceSpeed)) ForceSpeed(temp);
					_stopped = false;
				}
			}

			__enable_irq();
			return returnCode;
		}

		// Set new AlignWalls task (use last end state to start)
		template<>
		ReturnCode setNewTask<NewStateType::lastEndState>(const AlignWalls& newTask, const bool forceOverride)
		{
			static RobotState endState;
			static ReturnCode returnCode;
			static AlignWalls temp;

			returnCode = ReturnCode::ok;

			__disable_irq();

			if (_currentTask->isFinished() || forceOverride)
			{
				endState = static_cast<RobotState>(_currentTask->getEndState());

				temp = newTask;
				returnCode = temp.startTask(endState);

				if (returnCode == ReturnCode::ok)
				{
					_currentTask = new (&(_taskCopies.alignWalls)) AlignWalls(temp);
					_stopped = false;
				}
			}

			__enable_irq();
			return returnCode;
		}

		// Set new AlignWalls task (use current state to start)
		template<>
		ReturnCode setNewTask<NewStateType::currentState>(const AlignWalls& newTask, const bool forceOverride)
		{
			static ReturnCode returnCode;
			static AlignWalls temp;

			returnCode = ReturnCode::ok;

			__disable_irq();

			if (_currentTask->isFinished() || forceOverride)
			{
				temp = newTask;
				returnCode = temp.startTask(SensorFusion::getFusedData().robotState);

				if (returnCode == ReturnCode::ok)
				{
					_currentTask = new (&(_taskCopies.alignWalls)) AlignWalls(temp);
					_stopped = false;
				}
			}

			__enable_irq();
			return returnCode;
		}

		// Set new AlignWalls task (use specified state to start)
		ReturnCode setNewTask(const AlignWalls& newTask, RobotState startState, const bool forceOverride)
		{
			static ReturnCode returnCode;
			static AlignWalls temp;

			returnCode = ReturnCode::ok;

			__disable_irq();

			if (_currentTask->isFinished() || forceOverride)
			{
				temp = newTask;
				returnCode = temp.startTask(startState);

				if (returnCode == ReturnCode::ok)
				{
					_currentTask = new (&(_taskCopies.alignWalls)) AlignWalls(temp);
					_stopped = false;
				}
			}

			__enable_irq();
			return returnCode;
		}

		// Set new FollowWall task (use last end state to start)
		template<>
		ReturnCode setNewTask<NewStateType::lastEndState>(const FollowWall& newTask, const bool forceOverride)
		{
			static RobotState endState;
			static ReturnCode returnCode;
			static FollowWall temp;

			returnCode = ReturnCode::ok;

			__disable_irq();

			if (_currentTask->isFinished() || forceOverride)
			{
				endState = static_cast<RobotState>(_currentTask->getEndState());

				temp = newTask;
				returnCode = temp.startTask(endState);

				if (returnCode == ReturnCode::ok)
				{
					_currentTask = new (&(_taskCopies.followWall)) FollowWall(temp);
					_stopped = false;
				}
			}

			__enable_irq();
			return returnCode;
		}

		// Set new FollowWall task (use current state to start)
		template<>
		ReturnCode setNewTask<NewStateType::currentState>(const FollowWall& newTask, const bool forceOverride)
		{
			static ReturnCode returnCode;
			static FollowWall temp;

			returnCode = ReturnCode::ok;

			__disable_irq();

			if (_currentTask->isFinished() || forceOverride)
			{
				temp = newTask;
				returnCode = temp.startTask(SensorFusion::getFusedData().robotState);

				if (returnCode == ReturnCode::ok)
				{
					_currentTask = new (&(_taskCopies.followWall)) FollowWall(temp);
					_stopped = false;
				}
			}

			__enable_irq();
			return returnCode;
		}

		// Set new FollowWall task (use specified state to start)
		ReturnCode setNewTask(const FollowWall& newTask, RobotState startState, const bool forceOverride)
		{
			static ReturnCode returnCode;
			static FollowWall temp;

			returnCode = ReturnCode::ok;

			__disable_irq();

			if (_currentTask->isFinished() || forceOverride)
			{
				temp = newTask;
				returnCode = temp.startTask(startState);

				if (returnCode == ReturnCode::ok)
				{
					_currentTask = new (&(_taskCopies.followWall)) FollowWall(temp);
					_stopped = false;
				}
			}

			__enable_irq();
			return returnCode;
		}

		// Set new TaskArray task (use last end state to start)
		template<>
		ReturnCode setNewTask<NewStateType::lastEndState>(const TaskArray& newTask, const bool forceOverride)
		{
			static RobotState endState;
			static ReturnCode returnCode;

			returnCode = ReturnCode::ok;

			__disable_irq();

			if (_currentTask->isFinished() || forceOverride)
			{
				endState = static_cast<RobotState>(_currentTask->getEndState());

				TaskArray temp = newTask;
				returnCode = temp.startTask(endState);

				if (returnCode == ReturnCode::ok)
				{
					_currentTask = new (&(_taskCopies.taskArray)) TaskArray(temp);
					_stopped = false;
				}
			}

			__enable_irq();
			return returnCode;
		}

		// Set new TaskArray task (use current state to start)
		template<>
		ReturnCode setNewTask<NewStateType::currentState>(const TaskArray& newTask, const bool forceOverride)
		{
			static ReturnCode returnCode;

			returnCode = ReturnCode::ok;

			__disable_irq();

			if (_currentTask->isFinished() || forceOverride)
			{
				TaskArray temp = newTask;
				returnCode = temp.startTask(SensorFusion::getFusedData().robotState);

				if (returnCode == ReturnCode::ok)
				{
					_currentTask = new (&(_taskCopies.taskArray)) TaskArray(temp);
					_stopped = false;
				}
			}

			__enable_irq();
			return returnCode;
		}

		// Set new TaskArray task (use specified state to start)
		ReturnCode setNewTask(const TaskArray& newTask, RobotState startState, const bool forceOverride)
		{
			static ReturnCode returnCode;

			returnCode = ReturnCode::ok;

			__disable_irq();

			if (_currentTask->isFinished() || forceOverride)
			{
				TaskArray temp = newTask;
				returnCode = temp.startTask(startState);

				if (returnCode == ReturnCode::ok)
				{
					_currentTask = new (&(_taskCopies.taskArray)) TaskArray(temp);
					_stopped = false;
				}
			}

			__enable_irq();
			return returnCode;
		}

		// Is the current task finished?
		bool isTaskFinished()
		{
			return _currentTask->isFinished();
		}

		bool isDrivingStraight() {
			return _currentTask->isDrivingStraight();
		}

		void stopTask()
		{
			_stopped = true;
			setNewTask<NewStateType::currentState>(Stop(), true);
		}
	}
}