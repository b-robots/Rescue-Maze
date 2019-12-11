/*
This part of the Library is responsible for driving smoothly.
*/

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include "SmoothDriving_private.h"
#include "MotorControl_private.h"
#include "SensorFusion_private.h"

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

		DriveStraight::DriveStraight(uint8_t endSpeeds, float distance) : _endSpeeds(endSpeeds), _distance(distance), _targetAngle(SensorFusion::robotState.rotation.x) {}

		// Update speeds for both wheels
		Speeds DriveStraight::updateSpeeds(const uint8_t freq)
		{
			return Speeds{ 30, 30 };
		}

		// DriveStraight class - end

		// Rotate class - begin

		// Update speeds for both wheels
		Speeds Rotate::updateSpeeds(const uint8_t freq)
		{
			return Speeds{ 0, 0 };
		}

		// Rotate class - end

		// Update speeds for both wheels
		void updateSpeeds(const uint8_t freq)
		{
			auto updatedSpeeds = _currentTask->updateSpeeds(freq);

			MotorControl::setSpeed(MotorControl::Motor::left, updatedSpeeds.left);
			MotorControl::setSpeed(MotorControl::Motor::right, updatedSpeeds.right);
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