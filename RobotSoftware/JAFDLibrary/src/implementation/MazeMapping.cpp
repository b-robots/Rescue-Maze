/*
This part of the Library is responsible for mapping the maze and finding the shortest paths.
*/

#include "MazeMapping_private.h"
#include "../utility/SpiEeprom_private.h"
#include "../utility/Math_private.h"
#include <algorithm>


namespace JAFD
{
	namespace MazeMapping
	{
		// Anonymous namespace = private variables & methods
		namespace
		{
			// Class for handling the EEPROM
			SpiEeprom _spiEeprom;
		}

		// Home Position
		constexpr MapCoordinate homePosition = { 0, 0, 0 };

		// Start-Address
		constexpr uint32_t startAddress = 0;

		// Usable size for the maze mapping
		constexpr uint16_t usableSize = 64 * 1024;

		// Maximum/minimum coordinates that can fit in the SRAM
		constexpr int8_t maxX = 31;
		constexpr int8_t minX = -32;
		constexpr int8_t maxY = 31;
		constexpr int8_t minY = -32;

		// Comparison operators for MapCoordinate
		inline bool operator==(const MapCoordinate& lhs, const MapCoordinate& rhs) { return (lhs.floor == rhs.floor && lhs.x == rhs.x && lhs.y == rhs.y); }
		inline bool operator!=(const MapCoordinate& lhs, const MapCoordinate& rhs) { return !(lhs == rhs); }

		// Setup the MazeMapper
		ReturnCode mazeMapperSetup(MazeMapperSet settings)
		{
			_spiEeprom.init(settings.ramSSPin);

			return ReturnCode::ok;
		}
		
		// Set a grid cell in the RAM
		void setGridCell(GridCell gridCell, MapCoordinate coor)
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
			_spiEeprom.writeStream(address, bytes, 2);
		}

		// Read a grid cell from the RAM
		void getGridCell(GridCell* gridCell, MapCoordinate coor)
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
			_spiEeprom.readStream(address, bytes, 2);

			// Return data
			gridCell->cellConnections = bytes[0];
			gridCell->cellState = bytes[1];
		}

		// Set a grid cell in the RAM (only informations for the BF Algorithm)
		void setGridCell(uint8_t bfsValue, MapCoordinate coor)
		{
			// Memory address
			uint32_t address;
			
			// Calculate address
			address = ((coor.x + 0x20) & 0x3f) << 3;	// Bit 4 - 9 = x-Axis / 0 = 0x20
			address += ((coor.y + 0x20) & 0x3f) << 9;	// Bit 10 - 15 = y-Axis / 0 = 0x20
			address += ((coor.floor & 0x1) << 15);		// MSB (16.Bit) = floor
			address += 2;								// Go to the solver value

			// Write data
			_spiEeprom.writeByte(address, bfsValue);
		}

		// Read a grid cell from the RAM (only informations for the BF Algorithm)
		void getGridCell(uint8_t* bfsValue, MapCoordinate coor)
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
			*bfsValue = _spiEeprom.readByte(address);
		}

		// Set a grid cell in the RAM (including informations for the BF Algorithm)
		void setGridCell(GridCell gridCell, uint8_t bfsValue, MapCoordinate coor)
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
			_spiEeprom.writeStream(address, bytes, 3);
		}

		// Read a grid cell from the RAM (includeing informations for the BF Algorithm)
		void getGridCell(GridCell* gridCell, uint8_t* bfsValue, MapCoordinate coor)
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
			_spiEeprom.readStream(address, bytes, 3);

			// Return data
			gridCell->cellConnections = bytes[0];
			gridCell->cellState = bytes[1];
			*bfsValue = bytes[2];
		}
		
		// Reset all BFS Values
		void resetBFSValues(uint8_t floor)
		{
			for (int8_t x = minX; x <= maxX; x++)
			{
				for (int8_t y = minY; y <= maxY; y++)
				{
					setGridCell(0, { x, y, floor });
				}
			}
		}

		namespace BFAlgorithm
		{
			// Find the shortest known path from a to b
			ReturnCode findShortestPath(MapCoordinate start, uint8_t* directions, uint8_t maxPathLength, bool(*goalCondition)(MapCoordinate coor, GridCell cell))
			{
				static uint8_t lowerFloorID = 0;
				static uint8_t upperFloorID = 0;
				const uint8_t currentID = (start.floor == 0) ? lowerFloorID++ : upperFloorID++;

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

					if (goalCondition(coorV, gridCellV))
					{
						// Go the whole way backwards...
						while (coorV != start)
						{
							getGridCell(&bfsValueV, coorV);

							switch (bfsValueV & ~SolverState::discovered)
							{
							case SolverState::north:
								directions[distance++] = Direction::south; // Set the opposite direction
								coorV = { coorV.x, coorV.y + 1, coorV.floor };
								break;

							case SolverState::east:
								directions[distance++] = Direction::west; // Set the opposite direction
								coorV = { coorV.x + 1, coorV.y, coorV.floor };
								break;

							case SolverState::south:
								directions[distance++] = Direction::north; // Set the opposite direction
								coorV = { coorV.x, coorV.y - 1, coorV.floor };
								break;

							case SolverState::west:
								directions[distance++] = Direction::east; // Set the opposite direction
								coorV = { coorV.x - 1, coorV.y, coorV.floor };
								break;

							default:
								resetBFSValues(start.floor);
								return ReturnCode::error;
							}

							if (distance > maxPathLength)
							{
								resetBFSValues(start.floor);
								return ReturnCode::aborted;
							}
						}

						std::reverse(directions, directions + distance - 1);

						return ReturnCode::ok;
					}
					else
					{
						getGridCell(&gridCellV, coorV);

						// Check the north
						if (coorV.y < maxY && (gridCellV.cellConnections & Direction::north))
						{
							coorW = { coorV.x, coorV.y + 1, coorV.floor };
							getGridCell(&gridCellW, &bfsValueW, coorW);

							if (!(gridCellV.cellState & CellState::blackTile))
							{
								if (!(bfsValueW & SolverState::discovered))
								{
									bfsValueW = SolverState::discovered | SolverState::south; // Set discovered-bit, store the shortest path back and store the ID


									if (queue.enqueue(coorW) != ReturnCode::ok)
									{
										resetBFSValues(start.floor);
										return ReturnCode::error;
									}

									setGridCell(bfsValueW, coorW);
								}
							}
						}

						// Check the east
						if (coorV.x < maxX && (gridCellV.cellConnections & Direction::east))
						{
							coorW = { coorV.x + 1, coorV.y, coorV.floor };
							getGridCell(&gridCellW, &bfsValueW, coorW);

							if (!(gridCellV.cellState & CellState::blackTile))
							{
								if (!(bfsValueW & SolverState::discovered))
								{
									bfsValueW = SolverState::discovered | SolverState::west; // Set discovered-bit, store the shortest path back and store the ID


									if (queue.enqueue(coorW) != ReturnCode::ok)
									{
										resetBFSValues(start.floor);
										return ReturnCode::error;
									}

									setGridCell(bfsValueW, coorW);
								}
							}
						}

						// Check the south
						if (coorV.y > minY && (gridCellV.cellConnections & Direction::south))
						{
							coorW = { coorV.x, coorV.y - 1, coorV.floor };
							getGridCell(&gridCellW, &bfsValueW, coorW);

							if (!(gridCellV.cellState & CellState::blackTile))
							{
								if (!(bfsValueW & SolverState::discovered))
								{
									bfsValueW =  SolverState::discovered | SolverState::north; // Set discovered-bit, store the shortest path back and store the ID


									if (queue.enqueue(coorW) != ReturnCode::ok)
									{
										resetBFSValues(start.floor);
										return ReturnCode::error;
									}

									setGridCell(bfsValueW, coorW);
								}
							}
						}

						// Check the west
						if (coorV.x > minX && (gridCellV.cellConnections & Direction::west))
						{
							coorW = { coorV.x - 1, coorV.y, coorV.floor };
							getGridCell(&gridCellW, &bfsValueW, coorW);

							if (!(gridCellV.cellState & CellState::blackTile))
							{
								if (!(bfsValueW & SolverState::discovered))
								{
									bfsValueW = SolverState::discovered | SolverState::east; // Set discovered-bit, store the shortest path back and store the ID


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
				}

				resetBFSValues(start.floor);
				return ReturnCode::error;
			}
		}
	}
}
