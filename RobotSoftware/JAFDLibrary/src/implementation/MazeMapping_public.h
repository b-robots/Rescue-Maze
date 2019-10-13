/*
This public part of the Library is responsible for mapping the maze and finding the shortest paths.
*/

#pragma once

#include <stdint.h>

namespace JAFD
{
	// Namespace for the MazeMapper
	namespace MazeMapping
	{
		// Settings
		typedef struct {
			uint8_t ramSSPin; // Slave Select Pin of EEPROM
		} MazeMapperSet;
	}
}