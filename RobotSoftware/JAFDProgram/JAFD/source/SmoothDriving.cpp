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

			WheelSpeeds aaa;
		}

		// DriveStraight class - begin 

		DriveStraight::DriveStraight(int8_t endSpeeds, float distance) : _endSpeeds(endSpeeds), _endDistance(distance) {}

		void DriveStraight::startTask()
		{
			_targetDir = Vec2f(cosf(SensorFusion::getRobotState().rotation.x), sinf(SensorFusion::getRobotState().rotation.x));
			_startPos = Vec2f(SensorFusion::getRobotState().position.x, SensorFusion::getRobotState().position.y);
			_startSpeeds = (SensorFusion::getRobotState().wheelSpeeds.left + SensorFusion::getRobotState().wheelSpeeds.right) / 2;
			
			// Set start speeds to minimum value if distance is > 0
			if (_endDistance >= 0.01f)
			{
				if (_startSpeeds < JAFDSettings::MotorControl::minSpeed && _startSpeeds >= 0) _startSpeeds = JAFDSettings::MotorControl::minSpeed;
				else if (-_startSpeeds < JAFDSettings::MotorControl::minSpeed && _startSpeeds < 0) _startSpeeds = -JAFDSettings::MotorControl::minSpeed;
			}
		}

		// Update speeds for both wheels
		WheelSpeeds DriveStraight::updateSpeeds(const uint8_t freq)
		{
			volatile Vec2f test;

			test = Vec2f(0.0f, 0.0f);
			auto test2 = test + Vec2f(4.0f, 1.0f);
			volatile Vec2f test3 = test - test;
			test2 += 5.0f;
			test3 = test2 + test;
			test3 /= 1.0f;

			static Vec2f posRelToStart;		// Position relative to start position
			static Vec2f posError;			// Position error
			static float distance;			// Distance to start position
			static Vec2f integral;			// Position error integral
			static Vec2f corTerm;			// Correction term
			static Vec2f lastError;			// Last left speed
			static int8_t desiredSpeed;		// Wheel speeds
			static Vec2f driveVector;		// Speedvector the robot has to drive
			static float angularVel;		// Desired angular velocity

			// Calculate error
			posRelToStart = (Vec2f)SensorFusion::getRobotState().position - _startPos;
			distance = posRelToStart.length();
			posError = _targetDir * distance - posRelToStart;

			// Check if I am there
			if (distance - _endDistance >= 0.0f || abs(_endDistance) < 0.01f )
			{
				_finished = true;
				return WheelSpeeds{ _endSpeeds, _endSpeeds };
			}

			// PID controller
			corTerm = posError * _kp + integral * _ki - (lastError - posError) * _kd * (float)freq;

			integral += posError / (float)(freq);

			lastError = posError;

			// Accelerate / deccelerate
			desiredSpeed = _startSpeeds + static_cast<int8_t>((distance / _endDistance) * static_cast<float>(_endSpeeds - _startSpeeds));

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
			aaa = _currentTask->updateSpeeds(freq);
			MotorControl::setSpeeds(aaa);
		}
		WheelSpeeds getcalculatedspeeds()
		{
			return aaa;
		}
		// Set new task
		void setNewTask(const DriveStraight& newTask, const bool forceOverride)
		{
			if (_currentTask->_finished || forceOverride)
			{
				_taskCopies.straight = newTask;
				_currentTask = &_taskCopies.straight;
				_currentTask->startTask();
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