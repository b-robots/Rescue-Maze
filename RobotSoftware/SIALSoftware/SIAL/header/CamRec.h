#pragma once

#include "AllDatatypes.h"

namespace SIAL
{
	namespace CamRec
	{
		ReturnCode setup();

		void loop();

		Victim getVictim(bool left);

		extern bool dataReady;
	}
}