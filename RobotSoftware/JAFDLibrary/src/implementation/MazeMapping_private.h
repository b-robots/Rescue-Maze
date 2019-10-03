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
		// Cell Connections
		namespace CellConnections
		{
			constexpr uint8_t directionMask = 0xf;
			constexpr uint8_t rampMask = 0x10;
		}

		// Directions
		namespace Direction
		{
			constexpr uint8_t north = 1 << 0;
			constexpr uint8_t east = 1 << 1;
			constexpr uint8_t south = 1 << 2;
			constexpr uint8_t west = 1 << 3;
			constexpr uint8_t nowhere = 0;
		}

		// Ramp Direction
		namespace RampDirection
		{
			constexpr uint8_t north = 0b00 << 4;
			constexpr uint8_t east = 0b01 << 4;
			constexpr uint8_t south = 0b11 << 4;
			constexpr uint8_t west = 0b11 << 4;
		}

		// Possible states of a cell
		namespace CellState
		{
			constexpr uint8_t visited = 1 << 0;
			constexpr uint8_t victim = 1 << 1;
			constexpr uint8_t checkpoint = 1 << 2;
			constexpr uint8_t blackTile = 1 << 3;
			constexpr uint8_t ramp = 1 << 3;
		}

		// Value for the BFS - Algorithm
		namespace SolverState
		{
			constexpr uint8_t discovered = 1 << 0;
			constexpr uint8_t north = 0b00 << 1;
			constexpr uint8_t east = 0b01 << 1;
			constexpr uint8_t south = 0b10 << 1;
			constexpr uint8_t west = 0b11 << 1;
		}

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

		// Value for the BFS Algorithm
		typedef struct
		{
			// Information for the BFS Algorithm
			// 1.Bit: Already discovered?
			// 2. & 3. Bit: shortest path direction
			uint8_t solverState;

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
			ReturnCode findShortestPath(MapCoordinate a, uint8_t* directions, uint8_t maxPathLength, bool(*goalCondition)(MapCoordinate coor, GridCell cell));
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
