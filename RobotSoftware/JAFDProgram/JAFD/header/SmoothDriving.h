/*
This part of the Library is responsible for driving smoothly.
*/

#pragma once

#include "../../JAFDSettings.h"
#include "AllDatatypes.h"
#include "Vector.h"

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
			virtual ReturnCode startTask(RobotState startState) = 0;

			friend bool isTaskFinished();
			friend ReturnCode setNewTask(const DriveStraight& newTask, const bool forceOverride);
		protected:
			volatile bool _finished = false;							// Is the task already finished?
			volatile RobotState _endState;								// State of robot at the end of task
			ITask() = default;
		};

		class DriveStraight : public ITask
		{
		private:
			int16_t _endSpeeds;																// End speed of both wheels
			float _distance;																// Distance the robot has to travel
			Vec2f _targetDir;																// Target direction (guranteed to be normalized)
			Vec2f _startPos;																// Start position
			int16_t _startSpeeds;															// Average start speed of both wheels
			float _totalTime;																// Calculated time needed to drive
			static constexpr auto _kp = JAFDSettings::SmoothDriving::DriveStraight::kp;		// Kp factor for PID controller
			static constexpr auto _ki = JAFDSettings::SmoothDriving::DriveStraight::ki;		// Ki factor for PID controller
			static constexpr auto _kd = JAFDSettings::SmoothDriving::DriveStraight::kd;		// Kd factor for PID controller		
		public:
			DriveStraight(int16_t endSpeeds, float distance);
			ReturnCode startTask(RobotState startState);
			WheelSpeeds updateSpeeds(const uint8_t freq);
		};

		class Rotate : public ITask
		{
		private:
			float _angularVelocity;	// Angular velocity for rotation
			float _angle;			// Angle the robot has to rotate
		
		public:
			WheelSpeeds updateSpeeds(const uint8_t freq);
		};

		extern const DriveStraight Stop;

		void updateSpeeds(const uint8_t freq);													// Update speeds for both wheels
		ReturnCode setNewTask(const DriveStraight& newTask, const bool forceOverride = false);	// Set new task
		//void setNewTask(const Rotate& newTask, const bool forceOverride = false);				// Set new task
		bool isTaskFinished();																	// Is the current task finished?
	}
}