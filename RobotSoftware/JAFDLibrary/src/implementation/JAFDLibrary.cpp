/*
 Name:		RobotLibrary.cpp
 Created:	07.08.2019 13:20:59
 Author:	B.Robots
*/

#include "../JAFDLibrary.h"
#include "MazeMapping_private.h"
#include "Dispenser_private.h"

namespace JAFD
{
	// Just for testing...
	ReturnCode robotSetup(RobotSettings robotSettings)
	{
		// Setup of MazeMapper
		MazeMapping::mazeMapperSetup(robotSettings.mazeMapperSet);

		// Setup of Dispenser
		Dispenser::dispenserSetup(robotSettings.dispenserSet);

		return ReturnCode::ok;
	}

	ReturnCode robotLoop()
	{
		return ReturnCode::ok;
	}
}

