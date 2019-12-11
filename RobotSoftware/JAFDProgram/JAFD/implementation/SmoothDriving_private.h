/*
This part of the Library is responsible for driving smoothly.
*/

#pragma once

#include "ReturnCode_public.h"
#include <stdint.h>
#include <memory>

namespace JAFD
{
	namespace SmoothDriving
	{
		struct Speeds
		{
			uint8_t left;
			uint8_t right;
		};

		class ITask
		{
		public:
			bool finished;										// Is the task already finished?
			virtual Speeds getUpdatedSpeeds(uint8_t freq) = 0;	// Get updated speeds for both wheels
		};

		class DriveStraight : public ITask
		{
		private:
			uint8_t endSpeeds;	// End speed of both wheels
			float distance;		// Distance the robot has to travel
		public:
			Speeds getUpdatedSpeeds(const uint8_t freq);
		};

		void updateSpeeds(const uint8_t freq);							// Update speeds for both wheels
		void setNewTask(ITask& newTask, bool forceOverride = false);	// Set new task
	}
}