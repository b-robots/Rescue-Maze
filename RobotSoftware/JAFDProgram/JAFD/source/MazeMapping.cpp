/*
This part of the Library is responsible for mapping the maze and finding the shortest paths.
*/

#include "../header/MazeMapping.h"
#include "../header/SpiNVSRAM.h"
#include "../header/StaticQueue.h"
#include "../header/DistanceSensors.h"
#include "../header/SensorFusion.h"

#include <algorithm>

namespace JAFD
{
	namespace MazeMapping
	{
		// Setup the MazeMapper
		ReturnCode setup()
		{
			return ReturnCode::ok;
		}
		
		// Set a grid cell in the RAM
		void setGridCell(const GridCell gridCell, const MapCoordinate coor)
		{
			// Memory address
			uint32_t address;
			
			// Calculate address
			address = ((coor.x + 0x20) & 0x3f) << 3;	// Bit 4 - 9 = x-Axis / 0 = 0x20
			address += ((coor.y + 0x20) & 0x3f) << 9;	// Bit 10 - 15 = y-Axis / 0 = 0x20
			address += ((coor.floor & 0x1) << 15);		// MSB (16.Bit) = floor

			// Data as a byte array
			uint8_t bytes[2] = { gridCell.cellConnections, gridCell.cellState };

			// Write data
			SpiNVSRAM::writeStream(address, bytes, 2);
		}

		// Read a grid cell from the RAM
		void getGridCell(GridCell* gridCell, const MapCoordinate coor)
		{
			// Memory address
			uint32_t address;

			// Calculate address
			address = ((coor.x + 0x20) & 0x3f) << 3;	// Bit 4 - 9 = x-Axis / 0 = 0x20
			address += ((coor.y + 0x20) & 0x3f) << 9;	// Bit 10 - 15 = y-Axis / 0 = 0x20
			address += ((coor.floor & 0x1) << 15);		// MSB (16.Bit) = floor

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
			uint32_t address;
			
			// Calculate address
			address = ((coor.x + 0x20) & 0x3f) << 3;	// Bit 4 - 9 = x-Axis / 0 = 0x20
			address += ((coor.y + 0x20) & 0x3f) << 9;	// Bit 10 - 15 = y-Axis / 0 = 0x20
			address += ((coor.floor & 0x1) << 15);		// MSB (16.Bit) = floor
			address += 2;								// Go to the solver value

			// Write data
			SpiNVSRAM::writeByte(address, bfsValue);
		}

		// Read a grid cell from the RAM (only informations for the BF Algorithm)
		void getGridCell(uint8_t* bfsValue, const MapCoordinate coor)
		{
			// Memory address
			uint32_t address;

			// Calculate address
			address = ((coor.x + 0x20) & 0x3f) << 3;	// Bit 4 - 9 = x-Axis / 0 = 0x20
			address += ((coor.y + 0x20) & 0x3f) << 9;	// Bit 10 - 15 = y-Axis / 0 = 0x20
			address += ((coor.floor & 0x1) << 15);		// MSB (16.Bit) = floor
			address += 2;								// Go to the solver value

			// Data as a byte array
			uint8_t bytes[2];

			// Read data
			*bfsValue = SpiNVSRAM::readByte(address);
		}

		// Set a grid cell in the RAM (including informations for the BF Algorithm)
		void setGridCell(const GridCell gridCell, const uint8_t bfsValue, const MapCoordinate coor)
		{
			// Memory address
			uint32_t address;

			// Calculate address
			address = ((coor.x + 0x20) & 0x3f) << 3;	// Bit 4 - 9 = x-Axis / 0 = 0x20
			address += ((coor.y + 0x20) & 0x3f) << 9;	// Bit 10 - 15 = y-Axis / 0 = 0x20
			address += ((coor.floor & 0x1) << 15);		// MSB (16.Bit) = floor

			// Data as a byte array
			uint8_t bytes[3] = { gridCell.cellConnections, gridCell.cellState, bfsValue };

			// Write data
			SpiNVSRAM::writeStream(address, bytes, 3);
		}

		// Read a grid cell from the RAM (includeing informations for the BF Algorithm)
		void getGridCell(GridCell* gridCell, uint8_t* bfsValue, const MapCoordinate coor)
		{
			// Memory address
			uint32_t address;

			// Calculate address
			address = ((coor.x + 0x20) & 0x3f) << 3;	// Bit 4 - 9 = x-Axis / 0 = 0x20
			address += ((coor.y + 0x20) & 0x3f) << 9;	// Bit 10 - 15 = y-Axis / 0 = 0x20
			address += ((coor.floor & 0x1) << 15);		// MSB (16.Bit) = floor

			// Data as a byte array
			uint8_t bytes[3];

			// Read data
			SpiNVSRAM::readStream(address, bytes, 3);

			// Return data
			gridCell->cellConnections = bytes[0];
			gridCell->cellState = bytes[1];
			*bfsValue = bytes[2];
		}

		namespace BFAlgorithm
		{
			// Reset all BFS Values in this floor
			void resetBFSValues(const uint8_t floor)
			{
				for (int8_t x = minX; x <= maxX; x++)
				{
					for (int8_t y = minY; y <= maxY; y++)
					{
						setGridCell(0, { x, y, floor });
					}
				}
			}

			// Find the shortest known path from a to b
			ReturnCode findShortestPath(const MapCoordinate start, uint8_t* directions, const uint8_t maxPathLength, bool(*goalCondition)(MapCoordinate coor, GridCell cell))
			{
				static uint8_t lowerFloorID = 0;
				static uint8_t upperFloorID = 0;
				//const uint8_t currentID = (start.floor == 0) ? lowerFloorID++ : upperFloorID++;

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
					resetBFSValues(start.floor);
					return ReturnCode::error;
				}

				while (!queue.isEmpty())
				{
					if (queue.dequeue(&coorV) != ReturnCode::ok)
					{
						resetBFSValues(start.floor);
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
								directions[distance] = Direction::south; // Set the opposite direction
								coorV = { coorV.x, coorV.y + 1, coorV.floor };
								break;

							case SolverState::east:
								directions[distance] = Direction::west; // Set the opposite direction
								coorV = { coorV.x + 1, coorV.y, coorV.floor };
								break;

							case SolverState::south:
								directions[distance] = Direction::north; // Set the opposite direction
								coorV = { coorV.x, coorV.y - 1, coorV.floor };
								break;

							case SolverState::west:
								directions[distance] = Direction::east; // Set the opposite direction
								coorV = { coorV.x - 1, coorV.y, coorV.floor };
								break;

							default:
								resetBFSValues(start.floor);
								return ReturnCode::error;
							}

							distance++;

							if (distance > maxPathLength)
							{
								resetBFSValues(start.floor);
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
						if (coorV.y < maxY && (gridCellV.cellConnections & Direction::north))
						{
							coorW = { coorV.x, coorV.y + 1, coorV.floor };
							getGridCell(&gridCellW, &bfsValueW, coorW);

							if (!(bfsValueW & SolverState::discovered) && !(gridCellW.cellState & CellState::blackTile))
							{
								bfsValueW = SolverState::discovered | SolverState::south; // Set discovered-bit, store the shortest path back

								if (queue.enqueue(coorW) != ReturnCode::ok)
								{
									resetBFSValues(start.floor);
									return ReturnCode::error;
								}

								setGridCell(bfsValueW, coorW);
							}
						}

						// Check the east
						if (coorV.x < maxX && (gridCellV.cellConnections & Direction::east))
						{
							coorW = { coorV.x + 1, coorV.y, coorV.floor };
							getGridCell(&gridCellW, &bfsValueW, coorW);

							if (!(bfsValueW & SolverState::discovered) && !(gridCellW.cellState & CellState::blackTile))
							{
								bfsValueW = SolverState::discovered | SolverState::west; // Set discovered-bit, store the shortest path back

								if (queue.enqueue(coorW) != ReturnCode::ok)
								{
									resetBFSValues(start.floor);
									return ReturnCode::error;
								}

								setGridCell(bfsValueW, coorW);
							}
						}

						// Check the south
						if (coorV.y > minY && (gridCellV.cellConnections & Direction::south))
						{
							coorW = { coorV.x, coorV.y - 1, coorV.floor };
							getGridCell(&gridCellW, &bfsValueW, coorW);
							
							if (!(bfsValueW & SolverState::discovered) && !(gridCellW.cellState & CellState::blackTile))
							{
								bfsValueW = SolverState::discovered | SolverState::north; // Set discovered-bit, store the shortest path back

								if (queue.enqueue(coorW) != ReturnCode::ok)
								{
									resetBFSValues(start.floor);
									return ReturnCode::error;
								}

								setGridCell(bfsValueW, coorW);
							}
						}

						// Check the west
						if (coorV.x > minX && (gridCellV.cellConnections & Direction::west))
						{
							coorW = { coorV.x - 1, coorV.y, coorV.floor };
							getGridCell(&gridCellW, &bfsValueW, coorW);

							if (!(bfsValueW & SolverState::discovered) && !(gridCellW.cellState & CellState::blackTile))
							{
								bfsValueW = SolverState::discovered | SolverState::east; // Set discovered-bit, store the shortest path back

								if (queue.enqueue(coorW) != ReturnCode::ok)
								{
									resetBFSValues(start.floor);
									return ReturnCode::error;
								}

								setGridCell(bfsValueW, coorW);
							}
						}
					}
				}

				resetBFSValues(start.floor);
				return ReturnCode::error;
			}
		}

		// Update current GridCell
		void updateCurrentCell(volatile float& certainty, volatile GridCell& cell)
		{
			static MapCoordinate lastPosition;
			static float tempCertainty;
			GridCell tempCell;

			tempCertainty = 1.0f;

			lastPosition = MapCoordinate(SensorFusion::getFusedData().robotState.mapCoordinate);

			tempCell.cellConnections = Direction::north;

			/*if (DistanceSensors::frontLong.getDistance() > 30.0f - JAFDSettings::Mechanics::sensorFrontBackDist)
			{
				switch (SensorFusion::getFusedData().heading)
				{
				case HeadingDirection::north:
					tempCell.cellConnections |= Direction::north;
					break;
				case HeadingDirection::east:
					tempCell.cellConnections |= Direction::east;
					break;
				case HeadingDirection::south:
					tempCell.cellConnections |= Direction::south;
					break;
				case HeadingDirection::west:
					tempCell.cellConnections |= Direction::west;
					break;
				default:
					break;
				}
			}

			if (DistanceSensors::backLong.getDistance() > 30.0f - JAFDSettings::Mechanics::sensorFrontBackDist)
			{
				switch (SensorFusion::getFusedData().heading)
				{
				case HeadingDirection::north:
					tempCell.cellConnections |= Direction::south;
					break;
				case HeadingDirection::east:
					tempCell.cellConnections |= Direction::west;
					break;
				case HeadingDirection::south:
					tempCell.cellConnections |= Direction::north;
					break;
				case HeadingDirection::west:
					tempCell.cellConnections |= Direction::east;
					break;
				default:
					break;
				}
			}

			if (DistanceSensors::rightFront.getDistance() > 30.0f - JAFDSettings::Mechanics::sensorLeftRightDist)
			{
				switch (SensorFusion::getFusedData().heading)
				{
				case HeadingDirection::north:
					tempCell.cellConnections |= Direction::east;
					break;
				case HeadingDirection::east:
					tempCell.cellConnections |= Direction::south;
					break;
				case HeadingDirection::south:
					tempCell.cellConnections |= Direction::west;
					break;
				case HeadingDirection::west:
					tempCell.cellConnections |= Direction::north;
					break;
				default:
					break;
				}
			}

			if (DistanceSensors::leftFront.getDistance() > 30.0f - JAFDSettings::Mechanics::sensorLeftRightDist)
			{
				switch (SensorFusion::getFusedData().heading)
				{
				case HeadingDirection::north:
					tempCell.cellConnections |= Direction::west;
					break;
				case HeadingDirection::east:
					tempCell.cellConnections |= Direction::north;
					break;
				case HeadingDirection::south:
					tempCell.cellConnections |= Direction::east;
					break;
				case HeadingDirection::west:
					tempCell.cellConnections |= Direction::south;
					break;
				default:
					break;
				}
			}*/

			certainty = tempCertainty;
			cell = tempCell;
		}
	}
}
