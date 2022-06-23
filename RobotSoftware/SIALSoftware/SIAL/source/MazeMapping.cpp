#include "../header/MazeMapping.h"
#include "../header/SpiNVSRAM.h"
#include "../header/StaticQueue.h"
#include "../header/DistanceSensors.h"
#include "../header/SensorFusion.h"
#include "../SIALSettings.h"

#include <algorithm>
#include <cmath>

namespace SIAL
{
	namespace MazeMapping
	{
		// Setup the MazeMapper
		ReturnCode setup()
		{
			const uint8_t randVal1 = random(UINT8_MAX + 1);
			const uint8_t randVal2 = random(UINT8_MAX + 1);
			const uint8_t randBFVal = random(UINT8_MAX + 1);

			const MapCoordinate randCoor(random(minX, maxX + 1), random(minY, maxY + 1));

			const GridCell randomCell(randVal1, randVal2);

			setGridCell(randomCell, randBFVal, randCoor);

			GridCell readCell;
			uint8_t readBFVal;

			getGridCell(&readCell, &readBFVal, randCoor);

			resetAllCells();

			if (readCell.cellConnections != randomCell.cellConnections || readCell.cellState != randomCell.cellState || randBFVal != readBFVal)
			{
				return ReturnCode::error;
			}
			else
			{
				return ReturnCode::ok;
			}
		}

		void resetAllCells()
		{
			for (int8_t x = minX; x <= maxX; x++)
			{
				for (int8_t y = minY; y <= maxY; y++)
				{
					setGridCell(GridCell(), 0, MapCoordinate{ x, y });
				}
			}
		}

		// Set a grid cell in the RAM
		void setGridCell(const GridCell gridCell, const MapCoordinate coor)
		{
			// Memory address
			uint32_t address = SIALSettings::SpiNVSRAM::mazeMappingStartAddr;

			// Calculate address (MSB is unused)
			address += ((coor.x + 0x20) & 0x3f) << 3;	// Bit 4 - 9 = x-Axis / 0 = 0x20
			address += ((coor.y + 0x20) & 0x3f) << 9;	// Bit 10 - 15 = y-Axis / 0 = 0x20

			// Data as a byte array
			uint8_t bytes[2] = { gridCell.cellConnections, gridCell.cellState };

			// Write data
			SpiNVSRAM::writeStream(address, bytes, 2);
		}

		// Read a grid cell from the RAM
		void getGridCell(GridCell* gridCell, const MapCoordinate coor)
		{
			// Memory address
			uint32_t address = SIALSettings::SpiNVSRAM::mazeMappingStartAddr;

			// Calculate address
			address += ((coor.x + 0x20) & 0x3f) << 3;	// Bit 4 - 9 = x-Axis / 0 = 0x20
			address += ((coor.y + 0x20) & 0x3f) << 9;	// Bit 10 - 15 = y-Axis / 0 = 0x20

			// Data as a byte array
			uint8_t bytes[2];

			// Read data
			SpiNVSRAM::readStream(address, bytes, 2);

			// Return data
			gridCell->cellConnections = bytes[0];
			gridCell->cellState = bytes[1];
		}

		// Set a grid cell in the RAM (only informations for the BF Algorithm)
		void setGridCell(const uint8_t bfsValue, const MapCoordinate coor)
		{
			// Memory address
			uint32_t address = SIALSettings::SpiNVSRAM::mazeMappingStartAddr;

			// Calculate address
			address += ((coor.x + 0x20) & 0x3f) << 3;	// Bit 4 - 9 = x-Axis / 0 = 0x20
			address += ((coor.y + 0x20) & 0x3f) << 9;	// Bit 10 - 15 = y-Axis / 0 = 0x20
			address += 2;								// Go to the solver value

			// Write data
			SpiNVSRAM::writeByte(address, bfsValue);
		}

		// Read a grid cell from the RAM (only informations for the BF Algorithm)
		void getGridCell(uint8_t* bfsValue, const MapCoordinate coor)
		{
			// Memory address
			uint32_t address = SIALSettings::SpiNVSRAM::mazeMappingStartAddr;

			// Calculate address
			address += ((coor.x + 0x20) & 0x3f) << 3;	// Bit 4 - 9 = x-Axis / 0 = 0x20
			address += ((coor.y + 0x20) & 0x3f) << 9;	// Bit 10 - 15 = y-Axis / 0 = 0x20
			address += 2;								// Go to the solver value

			// Read data
			*bfsValue = SpiNVSRAM::readByte(address);
		}

		// Set a grid cell in the RAM (including informations for the BF Algorithm)
		void setGridCell(const GridCell gridCell, const uint8_t bfsValue, const MapCoordinate coor)
		{
			// Memory address
			uint32_t address = SIALSettings::SpiNVSRAM::mazeMappingStartAddr;

			// Calculate address
			address += ((coor.x + 0x20) & 0x3f) << 3;	// Bit 4 - 9 = x-Axis / 0 = 0x20
			address += ((coor.y + 0x20) & 0x3f) << 9;	// Bit 10 - 15 = y-Axis / 0 = 0x20

			// Data as a byte array
			uint8_t bytes[3] = { gridCell.cellConnections, gridCell.cellState, bfsValue };

			// Write data
			SpiNVSRAM::writeStream(address, bytes, 3);
		}

		// Read a grid cell from the RAM (includeing informations for the BF Algorithm)
		void getGridCell(GridCell* gridCell, uint8_t* bfsValue, const MapCoordinate coor)
		{
			// Memory address
			uint32_t address = SIALSettings::SpiNVSRAM::mazeMappingStartAddr;

			// Calculate address
			address += ((coor.x + 0x20) & 0x3f) << 3;	// Bit 4 - 9 = x-Axis / 0 = 0x20
			address += ((coor.y + 0x20) & 0x3f) << 9;	// Bit 10 - 15 = y-Axis / 0 = 0x20

			// Data as a byte array
			uint8_t bytes[3];

			// Read data
			SpiNVSRAM::readStream(address, bytes, 3);

			// Return data
			gridCell->cellConnections = bytes[0];
			gridCell->cellState = bytes[1];
			*bfsValue = bytes[2];
		}

		// Detect walls
		bool manageDetectedWalls(uint8_t frontWallsDetected, uint8_t leftWallsDetected, uint8_t rightWallsDetected, FusedData fusedData, GridCell& newCell, uint8_t& curSureWalls) {
			static MapCoordinate lastPosition = homePosition;
			static AbsoluteDir lastDir = AbsoluteDir::north;
			GridCell tempCell;
			tempCell.cellState = CellState::visited;
			tempCell.cellConnections = EntranceDirections::nowhere;
			uint8_t walls = 0b0000;											// Where are the walls; inverted to cellConnections

			uint8_t sureWalls = 0b0000;

			if (frontWallsDetected > 0)
			{
				switch (makeAbsolute(RelativeDir::forward, fusedData.robotState.heading))
				{

				case AbsoluteDir::north:
				{
					walls |= EntranceDirections::north;
					if (frontWallsDetected >= 2) {
						sureWalls |= EntranceDirections::north;
					}
					break;
				}

				case AbsoluteDir::east:
				{
					walls |= EntranceDirections::east;
					if (frontWallsDetected >= 2) {
						sureWalls |= EntranceDirections::east;
					}
					break;
				}

				case AbsoluteDir::south:
				{
					walls |= EntranceDirections::south;
					if (frontWallsDetected >= 2) {
						sureWalls |= EntranceDirections::south;
					}
					break;
				}

				case AbsoluteDir::west:
				{
					walls |= EntranceDirections::west;
					if (frontWallsDetected >= 2) {
						sureWalls |= EntranceDirections::west;
					}
					break;
				}

				default:
					break;
				}
			}

			if (leftWallsDetected > 0)
			{
				switch (makeAbsolute(RelativeDir::left, fusedData.robotState.heading))
				{

				case AbsoluteDir::north:
				{
					walls |= EntranceDirections::north;
					if (leftWallsDetected >= 2) {
						sureWalls |= EntranceDirections::north;
					}
					break;
				}

				case AbsoluteDir::east:
				{
					walls |= EntranceDirections::east;
					if (leftWallsDetected >= 2) {
						sureWalls |= EntranceDirections::east;
					}
					break;
				}

				case AbsoluteDir::south:
				{
					walls |= EntranceDirections::south;
					if (leftWallsDetected >= 2) {
						sureWalls |= EntranceDirections::south;
					}
					break;
				}

				case AbsoluteDir::west:
				{
					walls |= EntranceDirections::west;
					if (leftWallsDetected >= 2) {
						sureWalls |= EntranceDirections::west;
					}
					break;
				}

				default:
					break;
				}
			}

			if (rightWallsDetected > 0)
			{
				switch (makeAbsolute(RelativeDir::right, fusedData.robotState.heading))
				{

				case AbsoluteDir::north:
				{
					walls |= EntranceDirections::north;
					if (rightWallsDetected >= 2) {
						sureWalls |= EntranceDirections::north;
					}
					break;
				}

				case AbsoluteDir::east:
				{
					walls |= EntranceDirections::east;
					if (rightWallsDetected >= 2) {
						sureWalls |= EntranceDirections::east;
					}
					break;
				}

				case AbsoluteDir::south:
				{
					walls |= EntranceDirections::south;
					if (rightWallsDetected >= 2) {
						sureWalls |= EntranceDirections::south;
					}
					break;
				}

				case AbsoluteDir::west:
				{
					walls |= EntranceDirections::west;
					if (rightWallsDetected >= 2) {
						sureWalls |= EntranceDirections::west;
					}
					break;
				}

				default:
					break;
				}
			}

			if (fusedData.robotState.mapCoordinate == homePosition && lastPosition == homePosition && fusedData.robotState.heading != lastDir) {
				walls = walls | (~fusedData.gridCell.cellConnections & CellConnections::directionMask);
			}

			tempCell.cellConnections = (~walls) & CellConnections::directionMask;

			bool isOk = true;

			if (lastPosition == fusedData.robotState.mapCoordinate)
			{
				if (fusedData.robotState.mapCoordinate == homePosition) {
					constexpr uint8_t NEWmask = EntranceDirections::north | EntranceDirections::east | EntranceDirections::west;
					if ((tempCell.cellConnections & NEWmask) != (fusedData.gridCell.cellConnections & NEWmask)) {
						isOk = false;
					}
				}
				else
				{
					if ((tempCell.cellConnections & CellConnections::directionMask) != (fusedData.gridCell.cellConnections & CellConnections::directionMask)) {
						isOk = false;
					}
				}
			}

			if (lastPosition != fusedData.robotState.mapCoordinate || lastDir != fusedData.robotState.heading) {
				isOk = false;
			}

			lastPosition = fusedData.robotState.mapCoordinate;
			lastDir = fusedData.robotState.heading;

			newCell = tempCell;
			curSureWalls = sureWalls;

			return isOk;
		}

		namespace BFAlgorithm
		{
			// Reset all BFS Values in this floor
			void resetBFSValues()
			{
				for (int8_t x = minX; x <= maxX; x++)
				{
					for (int8_t y = minY; y <= maxY; y++)
					{
						setGridCell(0, MapCoordinate{ x, y });
					}
				}
			}

			// Find the shortest known path from a to b
			ReturnCode findShortestPath(const MapCoordinate start, uint8_t* directions, const uint8_t maxPathLength, bool(*goalCondition)(MapCoordinate coor, GridCell cell), bool(*isPassable)(GridCell cell))
			{
				StaticQueue<MapCoordinate, 64> queue; // MaxSize = 64, because this is enough for a normal labyrinth (4*63 would be maximum)

				GridCell gridCellV;
				uint8_t bfsValueV;
				MapCoordinate coorV;

				GridCell gridCellW;
				uint8_t bfsValueW;
				MapCoordinate coorW;

				uint8_t distance = 0;

				setGridCell(SolverState::discovered, start);

				if (queue.enqueue(start) != ReturnCode::ok)
				{
					resetBFSValues();
					return ReturnCode::error;
				}

				while (!queue.isEmpty())
				{
					if (queue.dequeue(&coorV) != ReturnCode::ok)
					{
						resetBFSValues();
						return ReturnCode::error;
					}

					getGridCell(&gridCellV, coorV);

					if (goalCondition(coorV, gridCellV))
					{
						// Go the whole way backwards...
						while (coorV != start)
						{
							getGridCell(&bfsValueV, coorV);

							switch (bfsValueV & ~SolverState::discovered)
							{
							case SolverState::north:
								directions[distance] = EntranceDirections::south; // Set the opposite direction
								coorV = MapCoordinate{ coorV.x, coorV.y + 1 };
								break;

							case SolverState::east:
								directions[distance] = EntranceDirections::west; // Set the opposite direction
								coorV = MapCoordinate{ coorV.x + 1, coorV.y };
								break;

							case SolverState::south:
								directions[distance] = EntranceDirections::north; // Set the opposite direction
								coorV = MapCoordinate{ coorV.x, coorV.y - 1 };
								break;

							case SolverState::west:
								directions[distance] = EntranceDirections::east; // Set the opposite direction
								coorV = MapCoordinate{ coorV.x - 1, coorV.y };
								break;

							default:
								resetBFSValues();
								return ReturnCode::error;
							}

							distance++;

							if (distance > maxPathLength)
							{
								resetBFSValues();
								return ReturnCode::aborted;
							}
						}

						if (distance > 0)
						{
							std::reverse(directions, directions + distance - 1);
						}

						return ReturnCode::ok;
					}
					else
					{
						// Check the north
						if (coorV.y < maxY && ((gridCellV.cellConnections & EntranceDirections::north) || (gridCellV.cellConnections & RampDirections::north)))
						{
							coorW = MapCoordinate{ coorV.x, coorV.y + 1 };
							getGridCell(&gridCellW, &bfsValueW, coorW);

							if (!(bfsValueW & SolverState::discovered) && !(gridCellW.cellState & CellState::blackTile) && isPassable(gridCellW))
							{
								bfsValueW = SolverState::discovered | SolverState::south; // Set discovered-bit, store the shortest path back

								if (queue.enqueue(coorW) != ReturnCode::ok)
								{
									resetBFSValues();
									return ReturnCode::error;
								}

								setGridCell(bfsValueW, coorW);
							}
						}

						// Check the east
						if (coorV.x < maxX && ((gridCellV.cellConnections & EntranceDirections::east) || (gridCellV.cellConnections & RampDirections::east)))
						{
							coorW = MapCoordinate{ coorV.x + 1, coorV.y };
							getGridCell(&gridCellW, &bfsValueW, coorW);

							if (!(bfsValueW & SolverState::discovered) && !(gridCellW.cellState & CellState::blackTile) && isPassable(gridCellW))
							{
								bfsValueW = SolverState::discovered | SolverState::west; // Set discovered-bit, store the shortest path back

								if (queue.enqueue(coorW) != ReturnCode::ok)
								{
									resetBFSValues();
									return ReturnCode::error;
								}

								setGridCell(bfsValueW, coorW);
							}
						}

						// Check the south
						if (coorV.y > minY && ((gridCellV.cellConnections & EntranceDirections::south) || (gridCellV.cellConnections & RampDirections::south)))
						{
							coorW = MapCoordinate{ coorV.x, coorV.y - 1 };
							getGridCell(&gridCellW, &bfsValueW, coorW);

							if (!(bfsValueW & SolverState::discovered) && !(gridCellW.cellState & CellState::blackTile) && isPassable(gridCellW))
							{
								bfsValueW = SolverState::discovered | SolverState::north; // Set discovered-bit, store the shortest path back

								if (queue.enqueue(coorW) != ReturnCode::ok)
								{
									resetBFSValues();
									return ReturnCode::error;
								}

								setGridCell(bfsValueW, coorW);
							}
						}

						// Check the west
						if (coorV.x > minX && ((gridCellV.cellConnections & EntranceDirections::west) || (gridCellV.cellConnections & RampDirections::west)))
						{
							coorW = MapCoordinate{ coorV.x - 1, coorV.y };
							getGridCell(&gridCellW, &bfsValueW, coorW);

							if (!(bfsValueW & SolverState::discovered) && !(gridCellW.cellState & CellState::blackTile) && isPassable(gridCellW))
							{
								bfsValueW = SolverState::discovered | SolverState::east; // Set discovered-bit, store the shortest path back

								if (queue.enqueue(coorW) != ReturnCode::ok)
								{
									resetBFSValues();
									return ReturnCode::error;
								}

								setGridCell(bfsValueW, coorW);
							}
						}
					}
				}

				resetBFSValues();
				return ReturnCode::error;
			}
		}
	}
}