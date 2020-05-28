/*
This private part of the Library is responsible for mapping the maze and finding the shortest paths.
*/

#pragma once

#include "AllDatatypes.h"

namespace JAFD
{
	// Namespace for the MazeMapper
	namespace MazeMapping
	{
		// Value for the BFS - Algorithm
		namespace SolverState
		{
			constexpr uint8_t discovered = 1 << 0;
			constexpr uint8_t north = 0b00 << 1;
			constexpr uint8_t east = 0b01 << 1;
			constexpr uint8_t south = 0b10 << 1;
			constexpr uint8_t west = 0b11 << 1;
		}

		// Usable size for the maze mapping
		constexpr uint32_t usableSize = 64 * 1024;

		// Maximum/minimum coordinates that can fit in the SRAM
		constexpr int8_t maxX = 31;
		constexpr int8_t minX = -32;
		constexpr int8_t maxY = 31;
		constexpr int8_t minY = -32;

		// Namespace for the Breadth-First-Search-Algorithm to find the shortest Path
		namespace BFAlgorithm
		{
			// Reset all BFS Values in this floor
			void resetBFSValues(const uint8_t floor);

			// Find the shortest known path from a to b
			ReturnCode findShortestPath(const MapCoordinate start, uint8_t* directions, const uint8_t maxPathLength, bool(*goalCondition)(MapCoordinate coor, GridCell cell));
		}

		// Setup the MazeMapper
		ReturnCode setup();
		
		// Set a grid cell in the RAM
		void setGridCell(const GridCell gridCell, const MapCoordinate coor);
		void setGridCell(const uint8_t bfsValue, const MapCoordinate coor);
		void setGridCell(const GridCell gridCell, const uint8_t bfsValue, const MapCoordinate coor);

		// Read a grid cell from the RAM
		void getGridCell(GridCell* gridCell, const MapCoordinate coor);
		void getGridCell(uint8_t* bfsValue, const MapCoordinate coor);
		void getGridCell(GridCell* gridCell, uint8_t* bfsValue, const MapCoordinate coor);

		// Set current cell and recalculate certainty
		void setCurrentCell(const GridCell gridCell, float& currentCertainty, const float updateCertainty, MapCoordinate coor);
	}
}
