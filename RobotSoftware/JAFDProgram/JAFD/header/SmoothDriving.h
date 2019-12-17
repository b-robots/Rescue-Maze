/*
This part of the Library is responsible for driving smoothly.
*/

#pragma once

#include "../../JAFDSettings.h"
#include "Vector.h"
#include "AllDatatypes.h"

#include <stdint.h>

namespace JAFD
{
	namespace SmoothDriving
	{
		class DriveStraight;
		class Rotate;

		class ITask
		{
		public:
			virtual WheelSpeeds updateSpeeds(const uint8_t freq) = 0;	// Update speeds for both wheels
			virtual void startTask() = 0;

			friend bool isTaskFinished();
			friend void setNewTask(const DriveStraight& newTask, const bool forceOverride);
		protected:
			bool _finished = false;										// Is the task already finished?
		};

		class DriveStraight : public ITask
		{
		private:
			int8_t _endSpeeds;																// End speed of both wheels
			float _endDistance;																// Distance the robot has to travel
			Vec2f _targetDir;																// Target direction (guranteed to be normalized)
			Vec2f _startPos;																// Start position
			int8_t _startSpeeds;															// Average start speed of both wheels
			static constexpr auto _kp = JAFDSettings::SmoothDriving::DriveStraight::kp;		// Kp factor for PID controller
			static constexpr auto _ki = JAFDSettings::SmoothDriving::DriveStraight::ki;		// Ki factor for PID controller
			static constexpr auto _kd = JAFDSettings::SmoothDriving::DriveStraight::kd;		// Kd factor for PID controller		
		public:
			DriveStraight(int8_t endSpeeds, float distance);
			void startTask();
			WheelSpeeds updateSpeeds(const uint8_t freq);
		};
		
		static const DriveStraight Stop(0, 1.0f);

		class Rotate : public ITask
		{
		private:
			float _angularVelocity;	// Angular velocity for rotation
			float _angle;			// Angle the robot has to rotate
		
		public:
			WheelSpeeds updateSpeeds(const uint8_t freq);
		};

		void updateSpeeds(const uint8_t freq);												// Update speeds for both wheels
		void setNewTask(const DriveStraight& newTask, const bool forceOverride = false);	// Set new task
		//void setNewTask(const Rotate& newTask, const bool forceOverride = false);			// Set new task
		bool isTaskFinished();																// Is the current task finished?
		WheelSpeeds getcalculatedspeeds();
	}
}