/*
 Name:		JAFDLibrary.h
 Created:	07.08.2019 13:20:59
 Author:	B.Robots
 Version:	1.1
*/

#pragma once

#define JAFD_LIB_VERSION "1.1" 

// All public header files of Library
#include "implementation/Dispenser_public.h"
#include "implementation/ReturnCode_public.h"
#include "implementation/MazeMapping_public.h"
#include "utility/StaticQueue.h"

// Namespace for robot (including sensors, maze solving algorithm, and so on...)
// JAFTD = Just Ask For The Direction
namespace JAFTD
{
	typedef struct
	{
		Dispenser::DispenserSettings dispenserSet;
		MazeMapping::MazeMapperSet mazeMapperSet;
	} RobotSettings;

	// Setup & Loop for the Robot
	ReturnCode robotSetup(RobotSettings robotSettings);
	ReturnCode robotLoop();
};
