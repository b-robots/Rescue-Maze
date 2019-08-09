/*
 Name:		RobotLibrary.cpp
 Created:	07.08.2019 13:20:59
 Author:	B.Robots
*/

#include "RobotLibrary.h"

namespace JAFTD
{
	// Just for testing...
	ReturnState robotSetup(RobotSettings robotSettings)
	{
		// Setup of MazeMapper
		auto test = MazeMapping::mazeMapperSetup(robotSettings.mazeMapperSet);

		return ReturnState::ok;
	}

	ReturnState robotLoop()
	{
		return ReturnState::ok;
	}
}

