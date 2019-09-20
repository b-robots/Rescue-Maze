/*
 Name:		JAFDLibrary.cpp
 Created:	07.08.2019 13:20:59
 Author:	B.Robots
*/

#include "JAFDLibrary.h"
#include "implementation/MazeMapping_private.h"
#include "implementation/Dispenser_private.h"

namespace JAFTD
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

