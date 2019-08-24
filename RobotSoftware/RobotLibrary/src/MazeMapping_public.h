/*
This public part of the Library is responsible for mapping the maze and finding the shortest paths.
*/

#pragma once

#include "RobotLibraryIncludes.h"

namespace JAFTD
{
	// Namespace for the MazeMapper
	namespace MazeMapping
	{
		// Settings
		typedef struct {
			byte ramSSPin;
		} MazeMapperSet;
	}
}