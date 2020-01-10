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
		// Type of new state
		enum class NewStateType : uint8_t
		{
			currentState,
			lastEndState,
			test
		};

		class Accelerate;
		class Rotate;
		class DriveStraight;
		class Stop;

		// NOTE: Due to the default arguments in 'setNewTask' and the function being a template function the declaration providing the default argument has to be before the friend declaration!

		// Set new Accelerate task
		template<NewStateType stateType>
		ReturnCode setNewTask(const Accelerate& newTask, const bool forceOverride = false);

		ReturnCode setNewTask(const Accelerate& newTask, RobotState startState, const bool forceOverride = false);

		// Set new DriveStraight task
		template<NewStateType stateType>
		ReturnCode setNewTask(const DriveStraight& newTask, const bool forceOverride = false);

		ReturnCode setNewTask(const DriveStraight& newTask, RobotState startState, const bool forceOverride = false);

		// Set new Stop task
		template<NewStateType stateType>
		ReturnCode setNewTask(const Stop& newTask, const bool forceOverride = false);

		ReturnCode setNewTask(const Stop& newTask, RobotState startState, const bool forceOverride = false);

		// Set new Rotate task
		template<NewStateType stateType>
		ReturnCode setNewTask(const Rotate& newTask, const bool forceOverride = false);

		ReturnCode setNewTask(const Rotate& newTask, RobotState startState, const bool forceOverride = false);

		class ITask
		{
		public:
			virtual WheelSpeeds updateSpeeds(const uint8_t freq) = 0;	// Update speeds for both wheels
			virtual ReturnCode startTask(RobotState startState) = 0;

			// Friends

			friend bool isTaskFinished();

			template<NewStateType stateType>
			friend ReturnCode setNewTask(const Accelerate& newTask, const bool forceOverride);
			friend ReturnCode setNewTask(const Accelerate& newTask, RobotState startState, const bool forceOverride);

			template<NewStateType stateType>
			friend ReturnCode setNewTask(const DriveStraight& newTask, const bool forceOverride);
			friend ReturnCode setNewTask(const DriveStraight& newTask, RobotState startState, const bool forceOverride);

			template<NewStateType stateType>
			friend ReturnCode setNewTask(const Stop& newTask, const bool forceOverride);
			friend ReturnCode setNewTask(const Stop& newTask, RobotState startState, const bool forceOverride);

			template<NewStateType stateType>
			friend ReturnCode setNewTask(const Rotate& newTask, const bool forceOverride);
			friend ReturnCode setNewTask(const Rotate& newTask, RobotState startState, const bool forceOverride);

		protected:
			volatile bool _finished = false;							// Is the task already finished?
			volatile RobotState _endState;								// State of robot at the end of task
			ITask() = default;
		};

		class Accelerate : public ITask
		{
		private:
			int16_t _endSpeeds;					// End speed of both wheels
			float _distance;					// Distance the robot has to travel
			Vec2f _targetDir;					// Target direction (guranteed to be normalized)
			Vec2f _startPos;					// Start position
			int16_t _startSpeeds;				// Average start speed of both wheels
			float _totalTime;					// Calculated time needed to drive
		public:
			Accelerate(int16_t endSpeeds = 0, float distance = 0.0f);
			ReturnCode startTask(RobotState startState);
			WheelSpeeds updateSpeeds(const uint8_t freq);
		};

		class DriveStraight : public ITask
		{
		private:
			int16_t _speeds;					// Speeds of both wheels
			float _distance;					// Distance the robot has to travel
			Vec2f _targetDir;					// Target direction (guranteed to be normalized)
			Vec2f _startPos;					// Start position
		public:
			DriveStraight(float distance = 0);
			ReturnCode startTask(RobotState startState);
			WheelSpeeds updateSpeeds(const uint8_t freq);
		};

		class Stop : public ITask
		{
		public:
			ReturnCode startTask(RobotState startState);
			WheelSpeeds updateSpeeds(const uint8_t freq);
		};

		class Rotate : public ITask
		{
		private:
			float _maxAngularVel;			// Maximum angular velocity
			float _angle;					// Angle the robot has to rotate in rad
			float _startAngle;				// Angle at start
			float _totalTime;				// Calculated time needed to drive
			bool _accelerate;				// Still accelerating?	
		public:
			Rotate(float maxAngularVel = 0, float angle = 0.0f);			// Set angular velocity in rad/s and angle in degree
			ReturnCode startTask(RobotState startState);
			WheelSpeeds updateSpeeds(const uint8_t freq);
		};

		void updateSpeeds(const uint8_t freq);								// Update speeds for both wheels

		bool isTaskFinished();												// Is the current task finished?
	}
}