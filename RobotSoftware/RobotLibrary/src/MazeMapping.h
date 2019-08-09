/*
This part of the Library is responsible for mapping the maze and finding the shortest paths.
*/

#pragma once

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include "Helper.h"

namespace JAFTD
{
	// Namespace for the MazeMapper
	namespace MazeMapping
	{
		typedef struct {
			byte ramSSPin;
		} MazeMapperSet;

		// Anonymous namespace = private variables 
		namespace
		{
			byte ramSSPin;
		}

		ReturnState mazeMapperSetup(MazeMapperSet settings);
	}
}
