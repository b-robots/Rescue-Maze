/*
This part of the Library is responsible for mapping the maze and finding the shortest paths.
*/

#include "MazeMapping.h"
#include "Helper.h"

namespace JAFTD
{
	namespace MazeMapping
	{
		ReturnState mazeMapperSetup(MazeMapperSet settings)
		{
			ramSSPin = settings.ramSSPin;
			return ReturnState::ok;
		}
	}
}
