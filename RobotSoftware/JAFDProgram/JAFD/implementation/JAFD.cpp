/*
 Name:		RobotLibrary.cpp
 Created:	07.08.2019 13:20:59
 Author:	B.Robots
*/

#include "../JAFD.h"
#include "MazeMapping_private.h"
#include "Dispenser_private.h"
#include "MotorControl_private.h"
#include "SpiEeprom_private.h"

#include <SPI.h>

namespace JAFD
{
	// Just for testing...
	ReturnCode robotSetup()
	{
		// Setup the SPI-Bus
		SPI.begin();
		SPI.beginTransaction(SPISettings(10e+6, MSBFIRST, SPI_MODE0));

		// Setup of MazeMapper
		if (MazeMapping::mazeMapperSetup() != ReturnCode::ok)
		{
			return ReturnCode::fatalError;
		}

		// Setup of Dispenser
		if (Dispenser::dispenserSetup() != ReturnCode::ok)
		{
			return ReturnCode::fatalError;
		}

		// Setup of Motor Control
		if (MotorControl::motorControlSetup() != ReturnCode::ok)
		{
			return ReturnCode::fatalError;
		}

		// Setup of SPI EEPROM
		if (SpiEeprom::spiEepromSetup() != ReturnCode::ok)
		{
			return ReturnCode::fatalError;
		}

		return ReturnCode::ok;
	}

	ReturnCode robotLoop()
	{
		return ReturnCode::ok;
	}
}

