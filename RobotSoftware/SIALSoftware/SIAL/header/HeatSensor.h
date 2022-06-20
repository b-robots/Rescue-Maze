#pragma once

#include "AllDatatypes.h"

namespace SIAL
{
	namespace HeatSensor
	{
		ReturnCode setup();

		bool detectVictim(HeatSensorSide sensor);
	}
}