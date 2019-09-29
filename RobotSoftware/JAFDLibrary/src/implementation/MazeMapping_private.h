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
		// Directions
		enum class Direction : uint8_t
		{
			north = 1 << 0,
			east = 1 << 1,
			south = 1 << 2,
			west = 1 << 3,
			nowhere = 0
		};

		// Possible states of a cell
		enum class CellState : uint8_t
		{
			visited = 1 << 0,
			victim = 1 << 1,
			checkpoint = 1 << 2,
			blackTile = 1 << 3,
			ramp = 1 << 3
		};

		// Direction of ramp
		enum class RampDirection : uint8_t
		{
			north = 0b00,
			east = 0b01,
			south = 0b11,
			west = 0b11
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
			// 5. & 6.Bit: 00 = Ramp North; 01 = Ramp East; 10 = Ramp South; 11 = Ramp West
			// 7. & 8.Bit: Unused
			uint8_t cellConnections;

			// Information about Speed Bumps, Victims, Checkpoints...
			// From right to left:
			// 1.Bit: Is this Cell already visited?
			// 2.Bit: Victim already detected?
			// 3.Bit: Checkpoint?
			// 4.Bit: Black Tile?
			// 5.Bit: Ramp?
			uint8_t cellState;
		} GridCell;

		// Coordinate on the map
		typedef struct
		{
			int8_t x;
			int8_t y;
			uint8_t floor;
		} MapCoordinate;

		RampDirection getRampDirection(GridCell cell);

		// Namespace for the Breadth-First-Search-Algorithm to find the shortest Path
		namespace BFAlgorithm
		{
			enum class SolverValue : uint8_t
			{
				discovered = 1 << 7,
				north = 0b00,
				east = 0b01,
				south = 0b10,
				west = 0b11
			};

			// Clear all Solver-Values in the EEPROM
			void clearSolverValues(uint8_t floor);

			// Find the shortest known path from a to b
			ReturnCode findShortestPath(MapCoordinate a, Direction* directions, uint8_t maxPathLength, bool(*goalCondition)(MapCoordinate coor, GridCell cell));
		}

		// Setup the MazeMapper
		ReturnCode mazeMapperSetup(MazeMapperSet settings);

		// Set a grid cell in the RAM
		ReturnCode setGridCell(GridCell gridCell, MapCoordinate coor);
		ReturnCode setGridCell(BFAlgorithm::SolverValue solverValue, MapCoordinate coor);
		ReturnCode setGridCell(GridCell gridCell, BFAlgorithm::SolverValue bfValue, MapCoordinate coor);

		// Read a grid cell from the RAM
		ReturnCode getGridCell(GridCell* gridCell, MapCoordinate coor);
		ReturnCode getGridCell(BFAlgorithm::SolverValue* bfValue, MapCoordinate coor);
		ReturnCode getGridCell(GridCell* gridCell, BFAlgorithm::SolverValue* bfValue, MapCoordinate coor);
	}
}