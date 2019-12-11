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

namespace JAFD
{
	namespace SmoothDriving
	{
		namespace
		{
			ITask* currentTask = nullptr;	// Current task
		}

		// Get updated speeds for both wheels
		Speeds DriveStraight::getUpdatedSpeeds(const uint8_t freq)
		{
			return Speeds{0, 0};
		}

		// Update speeds for both wheels
		void updateSpeeds(const uint8_t freq)
		{
			auto updatedSpeeds = currentTask->getUpdatedSpeeds(freq);

			MotorControl::setSpeed(Motor::left, updatedSpeeds.left);
			MotorControl::setSpeed(Motor::right, updatedSpeeds.right);
		}

		// Set new task
		void setNewTask(ITask& newTask, bool forceOverride = false)
		{
			if (currentTask->finished || forceOverride)
			{
				currentTask = &newTask;
			}
		}
	}
}