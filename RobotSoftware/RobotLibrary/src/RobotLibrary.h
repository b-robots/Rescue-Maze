/*
 Name:		RobotLibrary.h
 Created:	07.08.2019 13:20:59
 Author:	B.Robots
 Version:	1.1
*/

#pragma once

#define ROBOT_LIB_VERSION "1.1" 

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

// All public header files of Library
#include "ReturnCode_public.h"
#include "MazeMapping_public.h"
#include "Dispenser_public.h"

// Namespace for robot (including sensors, maze solving algorithm, and so on...)
// JAFTD = Just Ask For The Direction (proposal of name modification: WSIG = Where Should I Go?) 
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
