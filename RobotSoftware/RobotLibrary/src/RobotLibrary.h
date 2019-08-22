/*
 Name:		RobotLibrary.h
 Created:	07.08.2019 13:20:59
 Author:	B.Robots
 Version:	1.0.0
*/

#pragma once

#define ROBOT_LIB_VERSION "1.0.0" 

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

// All Header files of Library
#include "MazeMapping.h"
#include "Helper.h"
#include "Dispenser.h"
#include "StaticQueue.h"

// Namespace for robot (including sensors, maze solving algorithm, and so on...)
// JAFTD = Just Ask For The Direction (proposal of name modification: WSIG = Where Should I Go?) 
namespace JAFTD
{
	typedef struct {
		internal::MazeMapping::MazeMapperSet mazeMapperSet;
	} RobotSettings;

	// Setup & Loop for the Robot
	ReturnCode robotSetup(RobotSettings robotSettings);
	ReturnCode robotLoop();
};
