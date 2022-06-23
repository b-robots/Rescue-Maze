#pragma once

#include "AllDatatypes.h"
#include "PIDController.h"

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
		public:
			WheelSpeeds updateSpeeds(float dt);
			void startTask(RobotState startState);
			// angle in rad
			Rotate(float angle, float maxAngularVel);
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

		void setNewTask(ITask* task, bool forceOverride = false);
		void updateSpeeds();
		DrivingTaskInformation getInformation();
	}
}