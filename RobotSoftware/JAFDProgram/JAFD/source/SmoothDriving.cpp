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

		DriveStraight::DriveStraight(uint8_t endSpeeds, float distance) : _endSpeeds(endSpeeds), _endDistance(distance), _targetDir(cosf(SensorFusion::robotState.rotation.x), sinf(SensorFusion::robotState.rotation.x)) {}

		// Update speeds for both wheels
		WheelSpeeds DriveStraight::updateSpeeds(const uint8_t freq)
		{
			return WheelSpeeds{ _endSpeeds, _endSpeeds };
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