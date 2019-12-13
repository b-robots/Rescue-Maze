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
				Rotate rotate;

				_TaskCopies() : straight(DriveStraight(0, 0.0f)) {};
			} _taskCopies;			

			ITask* _currentTask = &_taskCopies.straight;	// Current task
		}

		// DriveStraight class - begin 

		DriveStraight::DriveStraight(uint8_t endSpeeds, float distance) : _endSpeeds(endSpeeds), _endDistance(distance), _targetDir(cosf(SensorFusion::robotState.rotation.x), sinf(SensorFusion::robotState.rotation.x)), _startPos(SensorFusion::robotState.position.x, SensorFusion::robotState.position.y), _startSpeeds((SensorFusion::robotState.wheelSpeeds.left + SensorFusion::robotState.wheelSpeeds.right) / 2) {}

		// Update speeds for both wheels
		WheelSpeeds DriveStraight::updateSpeeds(const uint8_t freq)
		{
			static Vec2f posRelToStart;		// Position relative to start position
			static Vec2f posError;			// Position error
			static float distance;			// Distance to start position
			static Vec2f integral;			// Position error integral
			static Vec2f tempVal;			// Temporary value
			static Vec2f lastError;			// Last left speed

			// Calculate error
			posRelToStart = SensorFusion::robotState.position - _startPos;
			distance = posRelToStart.length();
			posError = _targetDir * distance - posRelToStart;


			// PID controller
			tempVal = posError * _kp + integral * _ki - (lastError - posError) * _kd * (float)freq;

			if (tempVal > _maxCorVal / _cmPSToPerc) lTempVal = _maxCorVal / _cmPSToPerc;
			else if (lTempVal < -_maxCorVal / _cmPSToPerc) lTempVal = -_maxCorVal / _cmPSToPerc;

			lTempVal += (float)_lDesSpeed;
			lTempVal *= _cmPSToPerc;

			if (lTempVal > 1.0f) lTempVal = 1.0f;
			else if (lTempVal < -1.0f) lTempVal = -1.0f;

			rTempVal = _kp * rError + _ki * rIntegral - _kd * (rLastSpeed - _rSpeed) * (float)freq;

			if (rTempVal > _maxCorVal / _cmPSToPerc) rTempVal = _maxCorVal / _cmPSToPerc;
			else if (rTempVal < -_maxCorVal / _cmPSToPerc) rTempVal = -_maxCorVal / _cmPSToPerc;

			rTempVal += (float)_rDesSpeed;
			rTempVal *= _cmPSToPerc;

			if (rTempVal > 1.0f) rTempVal = 1.0f;
			else if (rTempVal < -1.0f) rTempVal = -1.0f;


			lIntegral += lError / (float)(freq);
			rIntegral += rError / (float)(freq);

			lLastSpeed = _lSpeed;
			rLastSpeed = _lSpeed;
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
			auto updatedSpeeds = _currentTask->updateSpeeds(freq);

			MotorControl::setSpeed(Motor::left, updatedSpeeds.left);
			MotorControl::setSpeed(Motor::right, updatedSpeeds.right);
		}

		// Set new task
		void setNewTask(const DriveStraight& newTask, const bool forceOverride)
		{
			if (_currentTask->finished || forceOverride)
			{
				_taskCopies.straight = newTask;
				_currentTask = &_taskCopies.straight;
			}
		}

		// Set new task
		void setNewTask(const Rotate& newTask, const bool forceOverride)
		{
			if (_currentTask->finished || forceOverride)
			{
				_taskCopies.rotate = newTask;
				_currentTask = &_taskCopies.rotate;
			}
		}

		// Is the current task finished?
		bool isTaskFinished()
		{
			return _currentTask->finished;
		}
	}
}