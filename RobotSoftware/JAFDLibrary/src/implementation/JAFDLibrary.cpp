/*
 Name:		RobotLibrary.cpp
 Created:	07.08.2019 13:20:59
 Author:	B.Robots
*/

#include "../JAFDLibrary.h"
#include "MazeMapping_private.h"
#include "Dispenser_private.h"

#include <SPI.h>

namespace JAFD
{
	// Just for testing...
	ReturnCode robotSetup(RobotSettings robotSettings)
	{
		// Setup the SPI-Bus
		SPI.begin();
		SPI.beginTransaction(SPISettings(20e+6, MSBFIRST, SPI_MODE1));

		// Setup of MazeMapper
		//MazeMapping::mazeMapperSetup(robotSettings.mazeMapperSet);

		// Setup of Dispenser
		Dispenser::dispenserSetup(robotSettings.dispenserSet);

		return ReturnCode::ok;
	}

	ReturnCode robotLoop()
	{
		return ReturnCode::ok;
	}
}

