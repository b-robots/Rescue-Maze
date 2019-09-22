/*
This private part of the Library is responsible for mapping the maze and finding the shortest paths.
*/

#pragma once

#include "ReturnCode_public.h"
#include "MazeMapping_public.h"
#include "../utility/StaticQueue_private.h"

#include <stdint.h>

namespace JAFD
{
	// Namespace for the MazeMapper
	namespace MazeMapping
	{
		// Home Position
		extern const MapCoordinate homePosition;

		// The maximum/minimum coordinates that can fit in the SRAM
		extern const int8_t maxX;
		extern const int8_t minX;
		extern const int8_t maxY;
		extern const int8_t minY;

		// Directions (can't be a enum class)
		enum Direction : uint8_t
		{
			north = 1 << 0,
			east = 1 << 1,
			south = 1 << 2,
			west = 1 << 3,
			nowhere = 0
		};

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
			// From right to left:
			// 1.Bit: Is this Cell already visited?
			uint8_t cellState;
		} GridCell;

		// Coordinate on the map
		typedef struct
		{
			int8_t x;
			int8_t y;
			uint8_t floor;
		} MapCoordinate;

		// Namespace for the Breadth-First-Search-Algorithm to find the shortest Path
		namespace BFAlgorithm
		{
			// Find the shortest known path from a to b
			ReturnCode findShortestPath(MapCoordinate a, Direction* directions, uint8_t maxPathLength, bool(*goalCondition)(MapCoordinate coor, GridCell cell));
		}

		// Setup the MazeMapper
		ReturnCode mazeMapperSetup(MazeMapperSet settings);

		// Set a grid cell in the RAM
		ReturnCode setGridCell(GridCell gridCell, MapCoordinate coor);
		ReturnCode setGridCell(uint16_t bfValue, MapCoordinate coor);
		ReturnCode setGridCell(GridCell gridCell, uint16_t bfValue, MapCoordinate coor);

		// Read a grid cell from the RAM
		ReturnCode getGridCell(GridCell* gridCell, MapCoordinate coor);
		ReturnCode getGridCell(uint16_t* bfValue, MapCoordinate coor);
		ReturnCode getGridCell(GridCell* gridCell, uint16_t* bfValue, MapCoordinate coor);
	}
}