/*
This part of the Library is responsible for driving smoothly.
*/

#pragma once

#include <new>

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
			explicit Accelerate(int16_t endSpeeds = 0, float distance = 0.0f);
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
			explicit DriveStraight(float distance = 0);
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
			explicit Rotate(float maxAngularVel = 0, float angle = 0.0f);			// Set angular velocity in rad/s and angle in degree
			ReturnCode startTask(RobotState startState);
			WheelSpeeds updateSpeeds(const uint8_t freq);
		};

		class ForceSpeed : public ITask
		{
		private:
			int16_t _speeds;					// Speeds of both wheels
			float _distance;					// Distance the robot has to travel
			Vec2f _targetDir;					// Target direction (guranteed to be normalized)
			Vec2f _startPos;					// Start position
		public:
			explicit ForceSpeed(int16_t speed = 0, float distance = 0);
			ReturnCode startTask(RobotState startState);
			WheelSpeeds updateSpeeds(const uint8_t freq);
		};

		class AlignFront : public ITask
		{
		private:
			uint16_t _alignDist;		// Distance to align to wall in mm
		public:
			explicit AlignFront(uint16_t alignDist = 30);
			ReturnCode startTask(RobotState startState);
			WheelSpeeds updateSpeeds(const uint8_t freq);
		};

		class TaskArray : public ITask
		{
		private:
			enum class _TaskType : uint8_t
			{
				accelerate,
				straight,
				stop,
				rotate,
				forceSpeed,
				alignFront
			} _taskTypes[JAFDSettings::SmoothDriving::maxArrrayedTasks];

			union _TaskCopies
			{
				Accelerate accelerate;
				DriveStraight straight;
				Stop stop;
				Rotate rotate;
				ForceSpeed forceSpeed;
				AlignFront alignFront;

				_TaskCopies() : stop() {};
				~_TaskCopies() {}
			} _taskCopies[JAFDSettings::SmoothDriving::maxArrrayedTasks];

			ITask* _taskArray[JAFDSettings::SmoothDriving::maxArrrayedTasks];

			uint8_t _numTasks = 0;
			int16_t _currentTaskNum = 0;

		public:
			TaskArray() = delete;

			TaskArray(const TaskArray& taskArray);

			TaskArray(const Accelerate& task);
			TaskArray(const DriveStraight& task);
			TaskArray(const Stop& task);
			TaskArray(const Rotate& task);
			TaskArray(const ForceSpeed& task);
			TaskArray(const AlignFront& task);

			// Uses SFINAE to prohibit more than maximum arguments.
			template<typename ...Rem, typename = char[JAFDSettings::SmoothDriving::maxArrrayedTasks - sizeof ...(Rem)]>
			TaskArray(const Accelerate& task, const Rem&... rem) : TaskArray(rem...)
			{
				_taskTypes[_numTasks] = _TaskType::accelerate;
				_taskArray[_numTasks] = new(&(_taskCopies[_numTasks].accelerate)) Accelerate(task);
				_numTasks++;
			}

			// Uses SFINAE to prohibit more than maximum arguments.
			template<typename ...Rem, typename = char[JAFDSettings::SmoothDriving::maxArrrayedTasks - sizeof ...(Rem)]>
			TaskArray(const DriveStraight& task, const Rem&... rem) : TaskArray(rem...)
			{
				_taskTypes[_numTasks] = _TaskType::straight;
				_taskArray[_numTasks] = new(&(_taskCopies[_numTasks].straight)) DriveStraight(task);
				_numTasks++;
			}

			// Uses SFINAE to prohibit more than maximum arguments.
			template<typename ...Rem, typename = char[JAFDSettings::SmoothDriving::maxArrrayedTasks - sizeof ...(Rem)]>
			TaskArray(const Stop& task, const Rem&... rem) : TaskArray(rem...)
			{
				_taskTypes[_numTasks] = _TaskType::stop;
				_taskArray[_numTasks] = new(&(_taskCopies[_numTasks].stop)) Stop(task);
				_numTasks++;
			}

			// Uses SFINAE to prohibit more than maximum arguments.
			template<typename ...Rem, typename = char[JAFDSettings::SmoothDriving::maxArrrayedTasks - sizeof ...(Rem)]>
			TaskArray(const Rotate& task, const Rem&... rem) : TaskArray(rem...)
			{
				_taskTypes[_numTasks] = _TaskType::rotate;
				_taskArray[_numTasks] = new(&(_taskCopies[_numTasks].rotate)) Rotate(task);
				_numTasks++;
			}

			// Uses SFINAE to prohibit more than maximum arguments.
			template<typename ...Rem, typename = char[JAFDSettings::SmoothDriving::maxArrrayedTasks - sizeof ...(Rem)]>
			TaskArray(const ForceSpeed& task, const Rem&... rem) : TaskArray(rem...)
			{
				_taskTypes[_numTasks] = _TaskType::forceSpeed;
				_taskArray[_numTasks] = new(&(_taskCopies[_numTasks].forceSpeed)) ForceSpeed(task);
				_numTasks++;
			}

			// Uses SFINAE to prohibit more than maximum arguments.
			template<typename ...Rem, typename = char[JAFDSettings::SmoothDriving::maxArrrayedTasks - sizeof ...(Rem)]>
			TaskArray(const AlignFront& task, const Rem&... rem) : TaskArray(rem...)
			{
				_taskTypes[_numTasks] = _TaskType::alignFront;
				_taskArray[_numTasks] = new(&(_taskCopies[_numTasks].alignFront)) ForceSpeed(task);
				_numTasks++;
			}

			ReturnCode startTask(RobotState startState);
			WheelSpeeds updateSpeeds(const uint8_t freq);
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

		// Set new ForceSpeed task
		template<NewStateType stateType>
		ReturnCode setNewTask(const ForceSpeed& newTask, const bool forceOverride = false);

		template<>
		ReturnCode setNewTask<NewStateType::currentState>(const ForceSpeed& newTask, const bool forceOverride);

		template<>
		ReturnCode setNewTask<NewStateType::lastEndState>(const ForceSpeed& newTask, const bool forceOverride);

		ReturnCode setNewTask(const ForceSpeed& newTask, RobotState startState, const bool forceOverride = false);

		// Set new AlignFront task
		template<NewStateType stateType>
		ReturnCode setNewTask(const AlignFront& newTask, const bool forceOverride = false);

		template<>
		ReturnCode setNewTask<NewStateType::currentState>(const AlignFront& newTask, const bool forceOverride);

		template<>
		ReturnCode setNewTask<NewStateType::lastEndState>(const AlignFront& newTask, const bool forceOverride);

		ReturnCode setNewTask(const AlignFront& newTask, RobotState startState, const bool forceOverride = false);

		// Set new TaskArray task
		template<NewStateType stateType>
		ReturnCode setNewTask(const TaskArray& newTask, const bool forceOverride = false);

		template<>
		ReturnCode setNewTask<NewStateType::currentState>(const TaskArray& newTask, const bool forceOverride);

		template<>
		ReturnCode setNewTask<NewStateType::lastEndState>(const TaskArray& newTask, const bool forceOverride);

		ReturnCode setNewTask(const TaskArray& newTask, RobotState startState, const bool forceOverride = false);
	
		// Stop current task
		void stopTask();
	}
}