#pragma once

#include "AllDatatypes.h"

namespace SIAL
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

		// Setup the MazeMapper
		ReturnCode setup();

		// Reset stored maze
		void resetAllCells();

		// Set a grid cell in the RAM
		void setGridCell(const GridCell gridCell, const MapCoordinate coor);
		void setGridCell(const uint8_t bfsValue, const MapCoordinate coor);
		void setGridCell(const GridCell gridCell, const uint8_t bfsValue, const MapCoordinate coor);

		// Read a grid cell from the RAM
		void getGridCell(GridCell* gridCell, const MapCoordinate coor);
		void getGridCell(uint8_t* bfsValue, const MapCoordinate coor);
		void getGridCell(GridCell* gridCell, uint8_t* bfsValue, const MapCoordinate coor);

		// Detect walls and entrances
		bool manageDetectedWalls(uint8_t frontWallsDetected, uint8_t leftWallsDetected, uint8_t rightWallsDetected, FusedData outputFusedData, GridCell& newcell, uint8_t& sureWalls);
	
		// Namespace for the Breadth-First-Search-Algorithm to find the shortest Path
		namespace BFAlgorithm
		{
			namespace GoalFunctions {
				inline bool unvisited(MapCoordinate coor, GridCell cell) {
					return !(cell.cellState & CellState::visited);
				}

				inline bool unvisitedNeighbour(MapCoordinate coor, GridCell cell) {
					if (cell.cellConnections & EntranceDirections::north) {
						GridCell neighbour;
						getGridCell(&neighbour, coor.getCoordinateInDir(AbsoluteDir::north));
						if (!(neighbour.cellState & (CellState::visited | CellState::blackTile))) {
							return true;
						}
					}

					if (cell.cellConnections & EntranceDirections::east) {
						GridCell neighbour;
						getGridCell(&neighbour, coor.getCoordinateInDir(AbsoluteDir::east));
						if (!(neighbour.cellState & (CellState::visited | CellState::blackTile))) {
							return true;
						}
					}

					if (cell.cellConnections & EntranceDirections::south) {
						GridCell neighbour;
						getGridCell(&neighbour, coor.getCoordinateInDir(AbsoluteDir::south));
						if (!(neighbour.cellState & (CellState::visited | CellState::blackTile))) {
							return true;
						}
					}

					if (cell.cellConnections & EntranceDirections::west) {
						GridCell neighbour;
						getGridCell(&neighbour, coor.getCoordinateInDir(AbsoluteDir::west));
						if (!(neighbour.cellState & (CellState::visited | CellState::blackTile))) {
							return true;
						}
					}

					return false;
				}

				inline bool home(MapCoordinate coor, GridCell cell) {
					return coor == homePosition;
				}
			}

			// Reset all BFS Values in this floor
			void resetBFSValues();

			// Find the shortest known path from a to b
			ReturnCode findShortestPath(MapCoordinate start, AbsoluteDir directions[], uint8_t maxPathLength, uint8_t& pathLenOut, bool(*goalCondition)(MapCoordinate coor, GridCell cell));
		}

	}
}
