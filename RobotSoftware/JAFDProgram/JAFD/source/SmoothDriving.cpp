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

namespace JAFD
{
	namespace SmoothDriving
	{
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
				AlignFront alignFront;

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

		Accelerate::Accelerate(int16_t endSpeeds, float distance) : ITask(), _endSpeeds(endSpeeds), _distance(distance), _targetDir(1.0f, 0.0f){}

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
			static Vec2f currentPosition;		// Current position of robot
			static float currentHeading;		// Current heading of robot;
			static Vec2f posRelToStart;			// Position relative to start
			static float drivenDistance;		// Distance to startpoint (with correct sign for direction)
			static float radiant;				// Needed as temporary value for acceleration/decceleration
			static float calculatedTime;		// Calculated time based on driven distance
			static float desiredSpeed;			// Desired linear velocity (calculated by acceleration/decceleration calculation + PID controller)
			static float desAngularVel;			// Desired angular velocity (calculated by pure pursuit algorithm + PID controller)
			static float lookAheadDistance;		// Lookahead distance for pure pursuit algorithm
			static Vec2f goalPointGlobal;		// Goal-Point in global coordinate space for pure pursuit algorithm
			static Vec2f goalPointRobot;		// Goal-Point in robot coordinate space for pure pursuit algorithm
			static float desCurvature;			// Desired curvature (calculated by pure pursuit algorithm)
			static float correctedForwardVel;	// Corrected forward velocity
			static float correctedAngularVel;	// Corrected angular velocity
			static WheelSpeeds output;			// Speed output for both wheels

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

			// A variation of pure pursuits controller where the goal point is a lookahead distance on the path away (not a lookahead distance from the robot).
			// Furthermore, the lookahead distance is dynamically adapted to the speed
			// Calculate goal point
			lookAheadDistance = JAFDSettings::Controller::PurePursuit::lookAheadGain * desiredSpeed;

			if (lookAheadDistance < JAFDSettings::Controller::PurePursuit::minLookAheadDist && lookAheadDistance > -JAFDSettings::Controller::PurePursuit::minLookAheadDist) lookAheadDistance = JAFDSettings::Controller::PurePursuit::minLookAheadDist * sgn(desiredSpeed);

			goalPointGlobal = _startPos + _targetDir * (posRelToStart.length() * sgn(_distance) + lookAheadDistance);

			// Transform goal point to robot coordinates
			goalPointRobot.x = (goalPointGlobal.x - currentPosition.x)  * cosf(currentHeading) + (goalPointGlobal.y - currentPosition.y) * sinf(currentHeading);
			goalPointRobot.y = -(goalPointGlobal.x - currentPosition.x) * sinf(currentHeading) + (goalPointGlobal.y - currentPosition.y) * cosf(currentHeading);

			// Calculate curvature and angular velocity
			desCurvature = 2.0f * goalPointRobot.y / (goalPointRobot.x * goalPointRobot.x + goalPointRobot.y * goalPointRobot.y);
			
			if (desCurvature > JAFDSettings::Controller::PurePursuit::maxCurvature) desCurvature = JAFDSettings::Controller::PurePursuit::maxCurvature;
			else if (desCurvature < -JAFDSettings::Controller::PurePursuit::maxCurvature) desCurvature = -JAFDSettings::Controller::PurePursuit::maxCurvature;

			desAngularVel = desiredSpeed * desCurvature;

			// Kind of PID - controller
			correctedForwardVel = desiredSpeed * 0.8f + _forwardVelPID.process(desiredSpeed, tempRobotState.forwardVel, 1.0f / freq);
			correctedAngularVel = desAngularVel * 0.8f + _angularVelPID.process(desAngularVel, tempRobotState.angularVel.x, 1.0f / freq);

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
			static Vec2f currentPosition;		// Current position of robot
			static float currentHeading;		// Current heading of robot;
			static Vec2f posRelToStart;			// Position relative to start
			static float absDrivenDist;			// Absolute distance to startpoint
			static float radiant;				// Needed as temporary value for acceleration/decceleration
			static float calculatedTime;		// Calculated time based on driven distance
			static float desAngularVel;			// Desired angular velocity (calculated by pure pursuit algorithm + PID controller)
			static float lookAheadDistance;		// Lookahead distance for pure pursuit algorithm
			static Vec2f goalPointGlobal;		// Goal-Point in global coordinate space for pure pursuit algorithm
			static Vec2f goalPointRobot;		// Goal-Point in robot coordinate space for pure pursuit algorithm
			static float desCurvature;			// Desired curvature (calculated by pure pursuit algorithm)
			static WheelSpeeds output;			// Speed output for both wheels
			static float correctedForwardVel;	// Corrected forward velocity
			static float correctedAngularVel;	// Corrected angular velocity

			const auto tempRobotState = SensorFusion::getFusedData().robotState;

			currentPosition = (Vec2f)(tempRobotState.position);
			currentHeading = tempRobotState.globalHeading;

			// Calculate driven distance
			posRelToStart = currentPosition - _startPos;
			absDrivenDist = posRelToStart.length();

			// Check if I am there
			if (fabsf(absDrivenDist) >= fabsf(_distance))
			{
				_finished = true;
			}

			// A variation of pure pursuits controller where the goal point is a lookahead distance on the path away (not a lookahead distance from the robot).
			// Furthermore, the lookahead distance is dynamically adapted to the speed
			// Calculate goal point
			lookAheadDistance = JAFDSettings::Controller::PurePursuit::lookAheadGain * _speeds;
			lookAheadDistance = fabsf(lookAheadDistance);

			if (lookAheadDistance < JAFDSettings::Controller::PurePursuit::minLookAheadDist) lookAheadDistance = JAFDSettings::Controller::PurePursuit::minLookAheadDist;

			goalPointGlobal = _startPos + _targetDir * (absDrivenDist + lookAheadDistance);

			// Transform goal point to robot coordinates
			goalPointRobot.x = (goalPointGlobal.x - currentPosition.x)  * cosf(currentHeading) + (goalPointGlobal.y - currentPosition.y) * sinf(currentHeading);
			goalPointRobot.y = -(goalPointGlobal.x - currentPosition.x) * sinf(currentHeading) + (goalPointGlobal.y - currentPosition.y) * cosf(currentHeading);

			// Calculate curvature and angular velocity
			desCurvature = 2.0f * goalPointRobot.y / (goalPointRobot.x * goalPointRobot.x + goalPointRobot.y * goalPointRobot.y);

			if (desCurvature > JAFDSettings::Controller::PurePursuit::maxCurvature) desCurvature = JAFDSettings::Controller::PurePursuit::maxCurvature;
			else if (desCurvature < -JAFDSettings::Controller::PurePursuit::maxCurvature) desCurvature = -JAFDSettings::Controller::PurePursuit::maxCurvature;

			desAngularVel = _speeds * desCurvature;

			// Kind of PID - controller
			correctedForwardVel = _speeds * 0.8f + _forwardVelPID.process(_speeds, tempRobotState.forwardVel, 1.0f / freq);
			correctedAngularVel = desAngularVel * 0.8f + _angularVelPID.process(desAngularVel, tempRobotState.angularVel.x, 1.0f / freq);

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
			static float rotatedAngle;			// Rotated angle since start
			static float desAngularVel;			// Desired angular velocity
			static float correctedAngularVel;	// By PID Controller corrected angular velocity
			static WheelSpeeds output;			// Output

			const auto tempRobotState = SensorFusion::getFusedData().robotState;

			// Calculate rotated angle
			rotatedAngle = tempRobotState.globalHeading - _startAngle;

			// Check if I am there
			if (fabsf(rotatedAngle) >= fabsf(_angle))
			{
				_finished = true;

				_forwardVelPID.reset();
				_angularVelPID.reset();

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

		ForceSpeed::ForceSpeed(int16_t speed, float distance) : ITask(), _speeds(speed), _distance(distance), _targetDir(1.0f, 0.0f) {}

		ReturnCode ForceSpeed::startTask(RobotState startState)
		{
			_finished = false;
			_targetDir = Vec2f(cosf(startState.globalHeading), sinf(startState.globalHeading));
			_startPos = (Vec2f)(startState.position);

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
		WheelSpeeds ForceSpeed::updateSpeeds(const uint8_t freq)
		{
			static Vec2f currentPosition;		// Current position of robot
			static float currentHeading;		// Current heading of robot;
			static Vec2f posRelToStart;			// Position relative to start
			static float absDrivenDist;			// Absolute distance to startpoint
			static float radiant;				// Needed as temporary value for acceleration/decceleration
			static float calculatedTime;		// Calculated time based on driven distance
			static float desAngularVel;			// Desired angular velocity (calculated by pure pursuit algorithm + PID controller)
			static float lookAheadDistance;		// Lookahead distance for pure pursuit algorithm
			static Vec2f goalPointGlobal;		// Goal-Point in global coordinate space for pure pursuit algorithm
			static Vec2f goalPointRobot;		// Goal-Point in robot coordinate space for pure pursuit algorithm
			static float desCurvature;			// Desired curvature (calculated by pure pursuit algorithm)
			static WheelSpeeds output;			// Speed output for both wheels
			static float correctedForwardVel;	// Corrected forward velocity
			static float correctedAngularVel;	// Corrected angular velocity

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

			// A variation of pure pursuits controller where the goal point is a lookahead distance on the path away (not a lookahead distance from the robot).
			// Furthermore, the lookahead distance is dynamically adapted to the speed
			// Calculate goal point
			lookAheadDistance = JAFDSettings::Controller::PurePursuit::lookAheadGain * _speeds;
			lookAheadDistance = fabsf(lookAheadDistance);

			if (lookAheadDistance < JAFDSettings::Controller::PurePursuit::minLookAheadDist) lookAheadDistance = JAFDSettings::Controller::PurePursuit::minLookAheadDist;

			goalPointGlobal = _startPos + _targetDir * (absDrivenDist + lookAheadDistance);

			// Transform goal point to robot coordinates
			goalPointRobot.x = (goalPointGlobal.x - currentPosition.x)  * cosf(currentHeading) + (goalPointGlobal.y - currentPosition.y) * sinf(currentHeading);
			goalPointRobot.y = -(goalPointGlobal.x - currentPosition.x) * sinf(currentHeading) + (goalPointGlobal.y - currentPosition.y) * cosf(currentHeading);

			// Calculate curvature and angular velocity
			desCurvature = 2.0f * goalPointRobot.y / (goalPointRobot.x * goalPointRobot.x + goalPointRobot.y * goalPointRobot.y);

			if (desCurvature > JAFDSettings::Controller::PurePursuit::maxCurvature) desCurvature = JAFDSettings::Controller::PurePursuit::maxCurvature;
			else if (desCurvature < -JAFDSettings::Controller::PurePursuit::maxCurvature) desCurvature = -JAFDSettings::Controller::PurePursuit::maxCurvature;

			desAngularVel = _speeds * desCurvature;

			// Kind of PID - controller
			correctedForwardVel = _speeds * 0.8f + _forwardVelPID.process(_speeds, tempRobotState.forwardVel, 1.0f / freq);
			correctedAngularVel = desAngularVel * 0.8f + _angularVelPID.process(desAngularVel, tempRobotState.angularVel.x, 1.0f / freq);

			// Compute wheel speeds - v = (v_r + v_l) / 2; w = (v_r - v_l) / wheelDistance => v_l = v - w * wheelDistance / 2; v_r = v + w * wheelDistance / 2
			output = WheelSpeeds{ correctedForwardVel - JAFDSettings::Mechanics::wheelDistance * correctedAngularVel / 2.0f, correctedForwardVel + JAFDSettings::Mechanics::wheelDistance * correctedAngularVel / 2.0f };

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

		// ForceSpeed class - end

		// AlignFront class - begin

		AlignFront::AlignFront(uint16_t alignDist) : ITask(), _alignDist(alignDist) {}

		ReturnCode AlignFront::startTask(RobotState startState)
		{
			float headingOffset;

			_finished = false;

			if (_alignDist < JAFDSettings::SmoothDriving::minAlignDist) return ReturnCode::fatalError;

			_angularVelPID.reset();
			_forwardVelPID.reset();

			_endState.position.x = roundf(startState.position.x / JAFDSettings::Field::cellWidth) * JAFDSettings::Field::cellWidth;
			_endState.position.y = roundf(startState.position.y / JAFDSettings::Field::cellWidth) * JAFDSettings::Field::cellWidth;
			_endState.position.z = 0;

			float positiveAngle = startState.globalHeading;

			while (positiveAngle < 0.0f) positiveAngle += M_TWOPI;
			while (positiveAngle > M_TWOPI) positiveAngle -= M_TWOPI;

			if (positiveAngle > 315.0f * DEG_TO_RAD || positiveAngle <= 45.0f * DEG_TO_RAD)
			{
				headingOffset = startState.globalHeading;

				_endState.position.x += JAFDSettings::Field::cellWidth / 2.0f - _alignDist / 10.0f - JAFDSettings::Mechanics::distSensFrontBackDist / 2.0f;
			}
			else if (positiveAngle > 45.0f * DEG_TO_RAD && positiveAngle <= 135.0f * DEG_TO_RAD)
			{
				headingOffset = startState.globalHeading - M_PI_2;

				_endState.position.y += JAFDSettings::Field::cellWidth / 2.0f - _alignDist / 10.0f - JAFDSettings::Mechanics::distSensFrontBackDist / 2.0f;
			}
			else if (positiveAngle > 135.0f * DEG_TO_RAD && positiveAngle <= 225.0f * DEG_TO_RAD)
			{
				headingOffset = startState.globalHeading - M_PI;

				_endState.position.x -= JAFDSettings::Field::cellWidth / 2.0f - _alignDist / 10.0f - JAFDSettings::Mechanics::distSensFrontBackDist / 2.0f;
			}
			else
			{
				headingOffset = startState.globalHeading - M_PI_2 * 3;

				_endState.position.y -= JAFDSettings::Field::cellWidth / 2.0f - _alignDist / 10.0f - JAFDSettings::Mechanics::distSensFrontBackDist / 2.0f;
			}

			while (headingOffset < 0.0f) headingOffset += M_TWOPI;
			while (headingOffset > M_PI) headingOffset -= M_TWOPI;

			_endState.globalHeading -= headingOffset;

			_endState.wheelSpeeds = FloatWheelSpeeds{ 0.0f, 0.0f };
			_endState.forwardVel = static_cast<float>(0.0f);
			_endState.angularVel = Vec3f(0.0f, 0.0f, 0.0f);
			
			return ReturnCode::ok;
		}

		// Update speeds for both wheels
		// Check for distance sensor errors is missing.
		WheelSpeeds AlignFront::updateSpeeds(const uint8_t freq)
		{
			static WheelSpeeds output;
			const auto tempDistances = SensorFusion::getFusedData().distances;
			const auto tempDistSensStates = SensorFusion::getFusedData().distSensorState;

			if (_finished)
			{
				return WheelSpeeds{ 0,0 };
			}

			if (abs(tempDistances.frontLeft - _alignDist) < JAFDSettings::SmoothDriving::maxAlignDistError && abs(tempDistances.frontRight - _alignDist) < JAFDSettings::SmoothDriving::maxAlignDistError)
			{
				_finished = true;
				return WheelSpeeds{ 0,0 };
			}
			else
			{
				if (tempDistances.frontLeft != 0)
				{
					if (tempDistances.frontLeft > _alignDist + JAFDSettings::SmoothDriving::maxAlignDistError)
					{
						output.left = JAFDSettings::SmoothDriving::alignSpeed;
					}
					else if ((int32_t)tempDistances.frontLeft < (int32_t)_alignDist - (int32_t)JAFDSettings::SmoothDriving::maxAlignDistError)
					{
						output.left = -JAFDSettings::SmoothDriving::alignSpeed;
					}
					else
					{
						output.left = 0;
					}
				}
				else
				{
					output.left = 0;
				}

				if (tempDistances.frontRight != 0)
				{
					if (tempDistances.frontRight > _alignDist + JAFDSettings::SmoothDriving::maxAlignDistError)
					{
						output.right = JAFDSettings::SmoothDriving::alignSpeed;
					}
					else if ((int32_t)tempDistances.frontRight < (int32_t)_alignDist - (int32_t)JAFDSettings::SmoothDriving::maxAlignDistError)
					{
						output.right = -JAFDSettings::SmoothDriving::alignSpeed;
					}
					else
					{
						output.right = 0;
					}
				}
				else
				{
					output.right = 0;
				}
			}

			return output;
		}

		// AlignFront class - end

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

		TaskArray::TaskArray(const AlignFront& task) : ITask()
		{
			_taskTypes[_numTasks] = _TaskType::alignFront;
			_taskArray[_numTasks] = new(&(_taskCopies[_numTasks].alignFront)) AlignFront(task);
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
				else
				{
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

		// Set new AlignFront task (use last end state to start)
		template<>
		ReturnCode setNewTask<NewStateType::lastEndState>(const AlignFront& newTask, const bool forceOverride)
		{
			static RobotState endState;
			static ReturnCode returnCode;
			static AlignFront temp;

			returnCode = ReturnCode::ok;

			__disable_irq();

			if (_currentTask->isFinished() || forceOverride)
			{
				endState = static_cast<RobotState>(_currentTask->getEndState());

				temp = newTask;
				returnCode = temp.startTask(endState);

				if (returnCode == ReturnCode::ok)
				{
					_currentTask = new (&(_taskCopies.alignFront)) AlignFront(temp);
					_stopped = false;
				}
			}

			__enable_irq();
			return returnCode;
		}

		// Set new AlignFront task (use current state to start)
		template<>
		ReturnCode setNewTask<NewStateType::currentState>(const AlignFront& newTask, const bool forceOverride)
		{
			static ReturnCode returnCode;
			static AlignFront temp;

			returnCode = ReturnCode::ok;

			__disable_irq();

			if (_currentTask->isFinished() || forceOverride)
			{
				temp = newTask;
				returnCode = temp.startTask(SensorFusion::getFusedData().robotState);

				if (returnCode == ReturnCode::ok)
				{
					_currentTask = new (&(_taskCopies.alignFront)) AlignFront(temp);
					_stopped = false;
				}
			}

			__enable_irq();
			return returnCode;
		}

		// Set new AlignFront task (use specified state to start)
		ReturnCode setNewTask(const AlignFront& newTask, RobotState startState, const bool forceOverride)
		{
			static ReturnCode returnCode;
			static AlignFront temp;

			returnCode = ReturnCode::ok;

			__disable_irq();

			if (_currentTask->isFinished() || forceOverride)
			{
				temp = newTask;
				returnCode = temp.startTask(startState);

				if (returnCode == ReturnCode::ok)
				{
					_currentTask = new (&(_taskCopies.alignFront)) AlignFront(temp);
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

		void stopTask()
		{
			_stopped = true;
			setNewTask<NewStateType::currentState>(Stop(), true);
		}
	}
}