/*
This part of the Library is responsible for driving smoothly.
*/

#pragma once

#include "../../JAFDSettings.h"
#include "ReturnCode_public.h"
#include <stdint.h>

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
			bool finished = false;									// Is the task already finished?
			virtual Speeds updateSpeeds(const uint8_t freq) = 0;	// Update speeds for both wheels
		};

		class DriveStraight : public ITask
		{
		private:
			uint8_t _endSpeeds;		// End speed of both wheels
			float _distance;		// Distance the robot has to travel
			float _targetAngle;		// Target angle
			static constexpr auto _kp = JAFDSettings::SmoothDriving::DriveStraight::kp;					// Kp factor for PID controller
			static constexpr auto _ki = JAFDSettings::SmoothDriving::DriveStraight::ki;					// Ki factor for PID controller
			static constexpr auto _kd = JAFDSettings::SmoothDriving::DriveStraight::kd;					// Kd factor for PID controller
			static constexpr auto _maxCorVal = JAFDSettings::SmoothDriving::DriveStraight::maxCorVal;	// Maximum correction value for PID controller
		
		public:
			DriveStraight(uint8_t endSpeeds, float distance);
			Speeds updateSpeeds(const uint8_t freq);
		};

		class Rotate : public ITask
		{
		private:
			float _angularVelocity;	// Angular velocity for rotation
			float _angle;			// Angle the robot has to rotate
		
		public:
			Speeds updateSpeeds(const uint8_t freq);
		};

		void updateSpeeds(const uint8_t freq);												// Update speeds for both wheels
		void setNewTask(const DriveStraight& newTask, const bool forceOverride = false);	// Set new task
		void setNewTask(const Rotate& newTask, const bool forceOverride = false);			// Set new task
		bool isTaskFinished();																// Is the current task finished?
	}
}