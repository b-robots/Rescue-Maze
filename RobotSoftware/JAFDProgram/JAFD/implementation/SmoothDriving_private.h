/*
This part of the Library is responsible for driving smoothly.
*/

#pragma once

#include "ReturnCode_public.h"
#include <stdint.h>

namespace JAFD
{
	namespace SmoothDriving
	{
		enum class TaskType : uint8_t
		{
			curve,
			straight
		};

		class Task
		{
		public:
			TaskType type;
		};

		class StraightTask : public Task
		{

		};

		class CurveTask : public Task
		{

		};

		void updateSpeed(uint8_t freq);
	}
}