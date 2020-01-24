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
			lastEndState
		};

		class ITask
		{
		public:
			virtual WheelSpeeds updateSpeeds(const uint8_t freq) = 0;	// Update speeds for both wheels
			virtual ReturnCode startTask(RobotState startState) = 0;
			bool isFinished();
			RobotState getEndState();
			virtual ~ITask() = default;
		protected:
			volatile bool _finished;			// Is the task already finished?
			volatile RobotState _endState;		// State of robot at the end of task
			ITask();
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

		class TaskArray
		{
		private:
			union _TaskCopies
			{
				Accelerate accelerate;
				DriveStraight straight;
				Stop stop;
				Rotate rotate;

				_TaskCopies() : stop() {};
				~_TaskCopies() {}
			} _taskCopies[JAFDSettings::SmoothDriving::maxArrrayedTasks];

			ITask* _taskArray[JAFDSettings::SmoothDriving::maxArrrayedTasks];

		public:
			TaskArray() = delete;

			TaskArray(Accelerate task) {}

			template<typename ...Rest>
			TaskArray(Accelerate task, Rest... rest) : TaskArray(rest...) {}
		};

		void updateSpeeds(const uint8_t freq);								// Update speeds for both wheels

		bool isTaskFinished();												// Is the current task finished?

		// Set new Accelerate task
		template<NewStateType stateType>
		ReturnCode setNewTask(const Accelerate& newTask, const bool forceOverride = false);

		template<>
		ReturnCode setNewTask<NewStateType::currentState>(const Accelerate& newTask, const bool forceOverride);

		template<>
		ReturnCode setNewTask<NewStateType::lastEndState>(const Accelerate& newTask, const bool forceOverride);

		ReturnCode setNewTask(const Accelerate& newTask, RobotState startState, const bool forceOverride = false);

		// Set new DriveStraight task
		template<NewStateType stateType>
		ReturnCode setNewTask(const DriveStraight& newTask, const bool forceOverride = false);

		template<>
		ReturnCode setNewTask<NewStateType::currentState>(const DriveStraight& newTask, const bool forceOverride);

		template<>
		ReturnCode setNewTask<NewStateType::lastEndState>(const DriveStraight& newTask, const bool forceOverride);

		ReturnCode setNewTask(const DriveStraight& newTask, RobotState startState, const bool forceOverride = false);

		// Set new Stop task
		template<NewStateType stateType>
		ReturnCode setNewTask(const Stop& newTask, const bool forceOverride = false);

		template<>
		ReturnCode setNewTask<NewStateType::currentState>(const Stop& newTask, const bool forceOverride);

		template<>
		ReturnCode setNewTask<NewStateType::lastEndState>(const Stop& newTask, const bool forceOverride);

		ReturnCode setNewTask(const Stop& newTask, RobotState startState, const bool forceOverride = false);

		// Set new Rotate task
		template<NewStateType stateType>
		ReturnCode setNewTask(const Rotate& newTask, const bool forceOverride = false);

		template<>
		ReturnCode setNewTask<NewStateType::currentState>(const Rotate& newTask, const bool forceOverride);

		template<>
		ReturnCode setNewTask<NewStateType::lastEndState>(const Rotate& newTask, const bool forceOverride);

		ReturnCode setNewTask(const Rotate& newTask, RobotState startState, const bool forceOverride = false);
	}
}