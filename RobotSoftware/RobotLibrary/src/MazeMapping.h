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
#include <SpiRAM.h>
#include <cstdint>

namespace JAFTD
{
	// Namespace for the MazeMapper
	namespace MazeMapping
	{
		// Settings
		typedef struct {
			byte ramSSPin;
		} MazeMapperSet;

		// Informations for one cell
		typedef struct
		{
			// Information about the entrances of the cell
			// From right to left:
			// 1.Bit: Entrance North
			// 2.Bit: Entrance East
			// 3.Bit: Entrance South
			// 4.Bit: Entrance West
			// 5.Bit: Is there a ramp?
			// 6. & 7.Bit: 00 = Ramp North; 01 = Ramp East; 10 = Ramp South; 11 = Ramp West
			// 8.Bit: Unused
			uint8_t cellConnections;

			// Information about Speed Bumps, Victims, Checkpoints...
			// Not used yet...
			uint8_t cellState;
		} GridCell;

		// Coordinate on the map
		typedef struct
		{
			int8_t x;
			int8_t y;
			uint8_t floor;
		} MapCoordinate;

		// Anonymous namespace = private variables & methods
		namespace
		{
			// SS Pin of RAM
			byte ramSSPin;

			// Class for handling the RAM
			SpiRAM spiRam(0, 0);
		}

		// Namespace for the Depth First algorithm to find the shortest Path
		namespace BFAlgorithm
		{
			// Find the shortest known path from a to b
			ReturnCode findShortestPath(MapCoordinate a, MapCoordinate goal, Direction* directions, uint8_t maxPathLength);
		}

		// Setup the MazeMapper
		ReturnCode mazeMapperSetup(MazeMapperSet settings);

		// Set a grid cell in the RAM
		ReturnCode setGridCell(GridCell gridCell, MapCoordinate coor);
		ReturnCode setGridCell(uint16_t solverValue, MapCoordinate coor);
		ReturnCode setGridCell(GridCell gridCell, uint16_t solverValue, MapCoordinate coor);

		// Read a grid cell from the RAM
		ReturnCode getGridCell(GridCell* gridCell, MapCoordinate coor);
		ReturnCode getGridCell(uint16_t* solverValue, MapCoordinate coor);
		ReturnCode getGridCell(GridCell* gridCell, uint16_t* solverValue, MapCoordinate coor);
	}
}
