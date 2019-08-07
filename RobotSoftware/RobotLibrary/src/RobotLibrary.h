/*
 Name:		RobotLibrary.h
 Created:	07.08.2019 13:20:59
 Author:	B.Robots
 Editor:	http://www.visualmicro.com
*/

#pragma once

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include <stdint.h>

// namespace for robot (including sensors, maze solving algorithm, and so on...)
// JAFTD = Just Ask For The Direction (proposal of name modification: WSIG = Where Should I Go?)
// Created: Patrick, 07.08.2019 
namespace JAFID
{
	// Return states
	// Can be extended when needed
	// Created: Patrick, 07.08.2019
	enum struct ReturnState : uint8_t
	{
		fatalError,
		error,
		aborted,
		ok
	};

	// Setup & Loop for the Robot
	ReturnState robotSetup();
	ReturnState robotLoop();
};
