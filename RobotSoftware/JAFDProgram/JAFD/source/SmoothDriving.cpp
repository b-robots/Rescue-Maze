/*
This part of the Library is responsible for driving smoothly.
*/

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include "../header/SmoothDriving.h"
#include "../header/MotorControl.h"
#include "../header/SensorFusion.h"
#include "../../JAFDSettings.h"
#include "../header/Math.h"

namespace JAFD
{
	namespace SmoothDriving
	{
		namespace
		{
			// Copy of current task
			union _TaskCopies
			{
				DriveStraight straight;
				//Rotate rotate;

				_TaskCopies() : straight(DriveStraight(0, 0.0f)) {};
			} _taskCopies;			

			ITask* _currentTask = &_taskCopies.straight;	// Current task
		}

		// DriveStraight class - begin 

		DriveStraight::DriveStraight(int16_t endSpeeds, float distance) : _endSpeeds(endSpeeds), _endDistance(distance) {}

		ReturnCode DriveStraight::startTask()
		{
			_targetDir = Vec2f(cosf(SensorFusion::getRobotState().rotation.x), sinf(SensorFusion::getRobotState().rotation.x));
			_startPos = Vec2f(SensorFusion::getRobotState().position.x, SensorFusion::getRobotState().position.y);
			_startSpeeds = (SensorFusion::getRobotState().wheelSpeeds.left + SensorFusion::getRobotState().wheelSpeeds.right) / 2;

			if (!((_startSpeeds >= 0 && _endSpeeds >= 0 && _endDistance >= 0) || (_startSpeeds <= 0 && _endSpeeds <= 0 && _endDistance <= 0))) return ReturnCode::error;
			
			if (_startSpeeds <= 0 && _endSpeeds <= 0)
			{
				_targetDir *= -1;
			}

			_totalTime = 2 * _endDistance / static_cast<float>(abs(_endSpeeds - _startSpeeds));

			return ReturnCode::ok;
		}

		// Update speeds for both wheels
		WheelSpeeds DriveStraight::updateSpeeds(const uint8_t freq)
		{
			static Vec2f posRelToStart;		// Position relative to start position
			static Vec2f posError;			// Position error
			static float distance;			// Distance to start position
			static Vec2f integral;			// Position error integral
			static Vec2f corTerm;			// Correction term
			static Vec2f lastError;			// Last left speed
			static int16_t desiredSpeed;		// Wheel speeds
			static Vec2f driveVector;		// Speedvector the robot has to drive
			static float angularVel;		// Desired angular velocity
			static float calculatedTime;	// Calculated time since task-start
			static float radiant;			// Radiant needed for calculation

			// Calculate error
			posRelToStart = (Vec2f)SensorFusion::getRobotState().position - _startPos;
			distance = posRelToStart.length();
			posError = _targetDir * distance - posRelToStart;

			// Check if I am there
			if (abs(distance) >= abs(_endDistance))
			{
				_finished = true;

				if (_endSpeeds < JAFDSettings::MotorControl::minSpeed && _endSpeeds >= 0) return WheelSpeeds{ JAFDSettings::MotorControl::minSpeed, JAFDSettings::MotorControl::minSpeed };
				else if (_endSpeeds > -JAFDSettings::MotorControl::minSpeed && _endSpeeds < 0) return WheelSpeeds{ -JAFDSettings::MotorControl::minSpeed, -JAFDSettings::MotorControl::minSpeed };
				else return WheelSpeeds{ _endSpeeds ,_endSpeeds };
			}

			// PID controller
			corTerm = posError * _kp + integral * _ki - (lastError - posError) * _kd * (float)freq;

			integral += posError / (float)(freq);

			lastError = posError;

			// Accelerate / deccelerate - v = v_1 + (t / t_ges) * (v_2 - v_1); s = integral(v * dt) = ((v_2 - v_1) * t^2) / (2 * t_ges) + v_1 * t => radiant = t_ges * v_1^2 - 2 * s * (v_1 + v_2); t = t_ges * (v_1 - sqrt(radiant)) / (v_2 - v_1); t_ges = s * 2 / (v_2 - v_1)
			radiant = static_cast<float>(_startSpeeds) * static_cast<float>(_startSpeeds) + 2.0f * distance * static_cast<float>(_endSpeeds - _startSpeeds) / _totalTime;
			calculatedTime = _totalTime * (static_cast<float>(_startSpeeds) - sqrtf(radiant)) / static_cast<float>(_startSpeeds - _endSpeeds);
			desiredSpeed = static_cast<float>(_startSpeeds) + (calculatedTime / _totalTime) * static_cast<float>(_endSpeeds - _startSpeeds);

			if (desiredSpeed < JAFDSettings::MotorControl::minSpeed && desiredSpeed >= 0) desiredSpeed = JAFDSettings::MotorControl::minSpeed;
			else if (desiredSpeed > -JAFDSettings::MotorControl::minSpeed && desiredSpeed < 0) desiredSpeed = -JAFDSettings::MotorControl::minSpeed;

			// Calculate drive vector
			driveVector = Vec2f(1.0f, 0.0f) + corTerm;

			// Calculate speeds - w = atan2(y, x); v = v_l / 2 + v_r / 2; w = v_l / (2 * b) - v_r / (2 * b); => v_l = b * w + v; v_r = v - b * w
			angularVel = atan2f(driveVector.y, driveVector.x);
			
			return WheelSpeeds{ desiredSpeed + JAFDSettings::Mechanics::wheelDistance * angularVel, desiredSpeed - JAFDSettings::Mechanics::wheelDistance * angularVel };
		}

		// DriveStraight class - end

		// Rotate class - begin

		// Update speeds for both wheels
		WheelSpeeds Rotate::updateSpeeds(const uint8_t freq)
		{
			return WheelSpeeds{ 0, 0 };
		}

		// Rotate class - end

		// Update speeds for both wheels
		void updateSpeeds(const uint8_t freq)
		{
			MotorControl::setSpeeds(_currentTask->updateSpeeds(freq));
		}

		// Set new task
		ReturnCode setNewTask(const DriveStraight& newTask, const bool forceOverride)
		{
			if (_currentTask->_finished || forceOverride)
			{
				_taskCopies.straight = newTask;
				_currentTask = &_taskCopies.straight;
				return _currentTask->startTask();
			}
		}

		// Set new task
		/*void setNewTask(const Rotate& newTask, const bool forceOverride)
		{
			if (_currentTask->_finished || forceOverride)
			{
				_taskCopies.rotate = newTask;
				_currentTask = &_taskCopies.rotate;
			}
		}*/

		// Is the current task finished?
		bool isTaskFinished()
		{
			return _currentTask->_finished;
		}
	}
}