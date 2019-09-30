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

		// Value for the BFS - Algorithm
		// 1.Bit: Already discovered?
		// 2. & 3. Bit: shortest path direction
		enum class SolverState : uint8_t
		{
			discovered = 1 << 0,
			north = 0b00 << 1,
			east = 0b01 << 1,
			south = 0b10 << 1,
			west = 0b11 << 1,
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
			CellState cellState;
		} GridCell;

		// Value for the BFS Algorithm
		typedef struct
		{
			// Information for the BFS Algorithm
			// 1.Bit: Already discovered?
			// 2. & 3. Bit: shortest path direction
			SolverState solverState;

			// Currrent ID
			uint8_t id;
		}BFSValue;

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
		void setGridCell(GridCell gridCell, MapCoordinate coor);
		void setGridCell(BFSValue bfsValue, MapCoordinate coor);
		void setGridCell(GridCell gridCell, BFSValue bfsValue, MapCoordinate coor);

		// Read a grid cell from the RAM
		void getGridCell(GridCell* gridCell, MapCoordinate coor);
		void getGridCell(BFSValue* bfsValue, MapCoordinate coor);
		void getGridCell(GridCell* gridCell, BFSValue* bfsValue, MapCoordinate coor);
	}
}
