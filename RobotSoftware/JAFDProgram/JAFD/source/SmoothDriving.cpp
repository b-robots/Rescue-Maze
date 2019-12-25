/*
This part of the Library is responsible for driving smoothly.
*/

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include <new>

#include "../header/SmoothDriving.h"
#include "../header/MotorControl.h"
#include "../header/SensorFusion.h"
#include "../../JAFDSettings.h"
#include "../header/Math.h"

namespace JAFD
{
	namespace SmoothDriving
	{
		namespace
		{
			// Copy of current task
			union _TaskCopies
			{
				Accelerate accelerate;
				DriveStraight straight;
				Stop stop;
				//Rotate rotate;

				_TaskCopies() : stop() {};
			} _taskCopies;			

			ITask* _currentTask = &_taskCopies.stop;	// Current task
		}

		// Accelerate class - begin 

		Accelerate::Accelerate(int16_t endSpeeds, float distance) : ITask(), _endSpeeds(endSpeeds), _distance(distance), _targetDir(1.0f, 0.0f), _startPos(0.0f, 0.0f), _startSpeeds(0), _totalTime(0) {}

		ReturnCode Accelerate::startTask(RobotState startState)
		{
			_finished = false;
			_targetDir = Vec2f(cosf(startState.rotation.x), sinf(startState.rotation.x));
			_startPos = (Vec2f)(startState.position);
			_startSpeeds = startState.forwardVel;

			if (!((_startSpeeds >= 0 && _endSpeeds >= 0 && _distance >= 0) || (_startSpeeds <= 0 && _endSpeeds <= 0 && _distance <= 0)) || _endSpeeds == _startSpeeds) return ReturnCode::error;
			
			if (_startSpeeds <= 0 && _endSpeeds <= 0)
			{
				_targetDir *= -1;
			}

			_totalTime = 2 * _distance / static_cast<float>(_endSpeeds + _startSpeeds);
			_totalTime = abs(_totalTime);

			_endState.wheelSpeeds = FloatWheelSpeeds{ _endSpeeds, _endSpeeds };
			_endState.forwardVel = static_cast<float>(_endSpeeds);
			_endState.position = startState.position + (Vec3f)(_targetDir * abs(_distance));
			_endState.angularVel = Vec3f(0.0f, 0.0f, 0.0f);
			_endState.rotation = startState.rotation;

			return ReturnCode::ok;
		}

		// Update speeds for both wheels
		WheelSpeeds Accelerate::updateSpeeds(const uint8_t freq)
		{
			static Vec2f posRelToStart;		// Position relative to start position
			static Vec2f posError;			// Position error
			static float distance;			// Distance to start position
			static Vec2f integral;			// Position error integral
			static Vec2f corTerm;			// Correction term
			static Vec2f lastError;			// Last left speed
			static int16_t desiredSpeed;	// Desired wheel speeds
			static Vec2f driveVector;		// Speedvector the robot has to drive
			static float angularVel;		// Desired angular velocity
			static float calculatedTime;	// Calculated time since task-start
			static float radiant;			// Radiant needed for calculation

			// Calculate error
			posRelToStart = (Vec2f)SensorFusion::getRobotState().position - _startPos;
			distance = posRelToStart.length() * sgn(_endSpeeds + _startSpeeds);
			posError = _targetDir * distance - posRelToStart;

			// Check if I am there
			if (abs(distance) >= abs(_distance))
			{
				_finished = true;	
				return WheelSpeeds{ _endSpeeds, _endSpeeds };
			}

			// PID controller
			corTerm = posError * _kp + integral * _ki - (lastError - posError) * _kd * (float)freq;

			integral += posError / (float)(freq);

			lastError = posError;

			// Accelerate / deccelerate - v = v_1 + (t / t_ges) * (v_2 - v_1); s = integral(v * dt) = ((v_2 - v_1) * t^2) / (2 * t_ges) + v_1 * t => radiant = t_ges * v_1^2 - 2 * s * (v_1 + v_2); t = t_ges * (v_1 - sqrt(radiant)) / (v_2 - v_1); t_ges = s * 2 / (v_2 - v_1)
			radiant = static_cast<float>(_startSpeeds) * static_cast<float>(_startSpeeds) + 2.0f * distance * static_cast<float>(_endSpeeds - _startSpeeds) / _totalTime;
			calculatedTime = _totalTime * (static_cast<float>(_startSpeeds) - sqrtf(abs(radiant)) * sgn(_startSpeeds + _endSpeeds)) / static_cast<float>(_startSpeeds - _endSpeeds);
			calculatedTime = abs(calculatedTime);
			desiredSpeed = static_cast<float>(_startSpeeds) + (calculatedTime / _totalTime) * static_cast<float>(_endSpeeds - _startSpeeds);
			
			// Correct speed if it is too low 
			if (desiredSpeed < JAFDSettings::MotorControl::minSpeed && desiredSpeed > -JAFDSettings::MotorControl::minSpeed) desiredSpeed = JAFDSettings::MotorControl::minSpeed * sgn(_distance);

			// Calculate drive vector
			driveVector = Vec2f(1.0f, 0.0f) + corTerm;

			// Calculate speeds - w = atan2(y, x); v = v_l / 2 + v_r / 2; w = v_l / (2 * b) - v_r / (2 * b); => v_l = b * w + v; v_r = v - b * w
			angularVel = atan2f(driveVector.y, driveVector.x);
			
			return WheelSpeeds{ desiredSpeed + JAFDSettings::Mechanics::wheelDistance * angularVel, desiredSpeed - JAFDSettings::Mechanics::wheelDistance * angularVel };
		}

		// Accelerate class - end

		// DriveStraight class - begin

		DriveStraight::DriveStraight(float distance) : ITask(), _speeds(0), _distance(distance), _targetDir(1.0f, 0.0f), _startPos(0.0f, 0.0f) {}

		ReturnCode DriveStraight::startTask(RobotState startState)
		{
			_finished = false;
			_targetDir = Vec2f(cosf(startState.rotation.x), sinf(startState.rotation.x));
			_startPos = (Vec2f)(startState.position);
			_speeds = startState.forwardVel;

			if (sgn(_speeds) != sgn(_distance)) return ReturnCode::error;

			if (_speeds <= 0)
			{
				_targetDir *= -1;
			}

			_endState.wheelSpeeds = FloatWheelSpeeds{ _speeds, _speeds };
			_endState.forwardVel = static_cast<float>(_speeds);
			_endState.position = startState.position + (Vec3f)(_targetDir * abs(_distance));
			_endState.angularVel = Vec3f(0.0f, 0.0f, 0.0f);
			_endState.rotation = startState.rotation;

			return ReturnCode::ok;
		}

		// Update speeds for both wheels
		WheelSpeeds DriveStraight::updateSpeeds(const uint8_t freq)
		{
			static Vec2f posRelToStart;		// Position relative to start position
			static Vec2f posError;			// Position error
			static float distance;			// Distance to start position
			static Vec2f integral;			// Position error integral
			static Vec2f corTerm;			// Correction term
			static Vec2f lastError;			// Last left speed
			static Vec2f driveVector;		// Speedvector the robot has to drive
			static float angularVel;		// Desired angular velocity

			// Calculate error
			posRelToStart = (Vec2f)SensorFusion::getRobotState().position - _startPos;
			distance = posRelToStart.length() * sgn(_speeds);
			posError = _targetDir * distance - posRelToStart;

			// Check if I am there
			if (abs(distance) >= abs(_distance))
			{
				_finished = true;
				return WheelSpeeds{ _speeds, _speeds };
			}

			// PID controller
			corTerm = posError * _kp + integral * _ki - (lastError - posError) * _kd * (float)freq;

			integral += posError / (float)(freq);

			lastError = posError;
			
			// Correct speed if it is too low 
			if (_speeds < JAFDSettings::MotorControl::minSpeed && _speeds > -JAFDSettings::MotorControl::minSpeed) _speeds = JAFDSettings::MotorControl::minSpeed * sgn(_distance);

			// Calculate drive vector
			driveVector = Vec2f(1.0f, 0.0f) + corTerm;

			// Calculate speeds - w = atan2(y, x); v = v_l / 2 + v_r / 2; w = v_l / (2 * b) - v_r / (2 * b); => v_l = b * w + v; v_r = v - b * w
			angularVel = atan2f(driveVector.y, driveVector.x);

			return WheelSpeeds{ _speeds + JAFDSettings::Mechanics::wheelDistance * angularVel, _speeds - JAFDSettings::Mechanics::wheelDistance * angularVel };
		}

		// DriveStraight class - end

		// Stop class - begin

		ReturnCode Stop::startTask(RobotState startState)
		{
			_finished = false;
			_endState.wheelSpeeds = FloatWheelSpeeds{ 0.0f, 0.0f };
			_endState.forwardVel = static_cast<float>(0.0f);
			_endState.position = startState.position;
			_endState.angularVel = Vec3f(0.0f, 0.0f, 0.0f);
			_endState.rotation = startState.rotation;
		}

		WheelSpeeds Stop::updateSpeeds(const uint8_t freq)
		{
			_finished = true;

			return WheelSpeeds{ 0, 0 };
		}

		// Stop class - end

		// Rotate class - begin

		// Update speeds for both wheels
		WheelSpeeds Rotate::updateSpeeds(const uint8_t freq)
		{
			return WheelSpeeds{ 0, 0 };
		}

		// Rotate class - end

		// Update speeds for both wheels
		void updateSpeeds(const uint8_t freq)
		{
			MotorControl::setSpeeds(_currentTask->updateSpeeds(freq));
		}

		// Set new task
		ReturnCode setNewTask(const Accelerate& newTask, const bool forceOverride)
		{
			static RobotState endState;
			static ReturnCode returnCode;
			static Accelerate temp;

			returnCode = ReturnCode::ok;

			__disable_irq();

			if (_currentTask->_finished || forceOverride)
			{
				endState = static_cast<decltype(endState)>(_currentTask->_endState);

				temp = newTask;
				returnCode = temp.startTask(endState);

				if (returnCode == ReturnCode::ok)
				{
					_currentTask = new (&(_taskCopies.accelerate)) Accelerate(temp);
				}
			}

			__enable_irq();
			return returnCode;
		}

		// Set new task
		ReturnCode setNewTask(const DriveStraight& newTask, const bool forceOverride)
		{
			static RobotState endState;
			static ReturnCode returnCode;
			static DriveStraight temp;

			returnCode = ReturnCode::ok;

			__disable_irq();

			if (_currentTask->_finished || forceOverride)
			{
				endState = static_cast<decltype(endState)>(_currentTask->_endState);
				
				temp = newTask;
				returnCode = temp.startTask(endState);

				if (returnCode == ReturnCode::ok)
				{
					_currentTask = new (&(_taskCopies.straight)) DriveStraight(temp);
				}
			}

			__enable_irq();
			return returnCode;
		}

		// Set new task
		ReturnCode setNewTask(const Stop& newTask, const bool forceOverride)
		{
			static RobotState endState;
			static Stop temp;

			__disable_irq();

			if (_currentTask->_finished || forceOverride)
			{
				endState = static_cast<decltype(endState)>(_currentTask->_endState);

				temp = newTask;
				temp.startTask(endState);

				_currentTask = new (&(_taskCopies.stop)) Stop(temp);
			}

			__enable_irq();
			return ReturnCode::ok;
		}

		// Set new task
		/*void setNewTask(const Rotate& newTask, const bool forceOverride)
		{
			if (_currentTask->_finished || forceOverride)
			{
				_taskCopies.rotate = newTask;
				_currentTask = &_taskCopies.rotate;
			}
		}*/

		// Is the current task finished?
		bool isTaskFinished()
		{
			return _currentTask->_finished;
		}
	}
}