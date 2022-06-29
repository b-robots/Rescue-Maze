#pragma once

#include "AllDatatypes.h"
#include "PIDController.h"

#include <initializer_list>

namespace SIAL {
	namespace SmoothDriving {
		class ITask {
		public:
			virtual WheelSpeeds updateSpeeds(float dt) = 0;
			virtual void startTask(RobotState startState) = 0;
			DrivingTaskInformation getInformation();
			virtual ~ITask() = default;
		protected:
			DrivingTaskInformation _information;
			ITask(DrivingTaskInformation information);
		};

		class Stop : public ITask {
		public:
			WheelSpeeds updateSpeeds(float dt);
			void startTask(RobotState startState);
			Stop();
		};

		class ForceSpeed : public ITask {
		private:
			int32_t _dist;
			int16_t _speed;
			Vec3f _startPos;
		public:
			WheelSpeeds updateSpeeds(float dt);
			void startTask(RobotState startState);
			// dist in cm
			ForceSpeed(int32_t dist, int16_t speed);
		};

		class Rotate : public ITask {
		private:
			float _angle;
			float _maxAngularVel;
			float _startAngle;
			float _totalTime;
			bool _accelerate;
			bool _snapOrientation;
		public:
			WheelSpeeds updateSpeeds(float dt);
			void startTask(RobotState startState);
			// angle in rad
			Rotate(float angle, float maxAngularVel, bool snapOrientation = true);
		};

		class FollowWall : public ITask {
		private:
			int32_t _dist;
			int16_t _speed;
			Vec3f _startPos;
			PIDController _pid;
		public:
			WheelSpeeds updateSpeeds(float dt);
			void startTask(RobotState startState);
			FollowWall(int32_t dist, int16_t speed);
		};

		class FollowCell : public ITask {
		private:
			FollowWall _wallTask;
			int16_t _speed;
		public:
			WheelSpeeds updateSpeeds(float dt);
			void startTask(RobotState startState);
			FollowCell(int16_t speed);
		};

		class AlignWalls : public ITask
		{
		private:
			float _absAvgAngle;
			bool _first;
			float _time;
			uint8_t _consOk;
		public:
			WheelSpeeds updateSpeeds(float dt);
			void startTask(RobotState startState);
			AlignWalls();
		};

		class TaskArray : public ITask
		{
		private:
			uint8_t _num;
			uint8_t _i;
			ITask** _tasks;
		public:
			WheelSpeeds updateSpeeds(float dt);
			void startTask(RobotState startState);
			TaskArray(std::initializer_list<SmoothDriving::ITask*> tasks);
			~TaskArray();
		};

		void startNewTask(ITask* task, bool forceOverride = false);
		void updateSpeeds();
		DrivingTaskInformation getInformation();
		void stop();
	}
}