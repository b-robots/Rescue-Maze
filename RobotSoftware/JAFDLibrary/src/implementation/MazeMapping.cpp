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
		void setGridCell(BFSValue bfsValue, MapCoordinate coor)
		{
			// Memory address
			uint32_t address;
			
			// Calculate address
			address = ((coor.x + 0x20) & 0x3f) << 3;	// Bit 4 - 9 = x-Axis / 0 = 0x20
			address += ((coor.y + 0x20) & 0x3f) << 9;	// Bit 10 - 15 = y-Axis / 0 = 0x20
			address += ((coor.floor & 0x1) << 15);		// MSB (16.Bit) = floor
			address += 2;								// Go to the solver value

			// Data as a byte array
			uint8_t bytes[2] = { bfsValue.solverState, bfsValue.id };

			// Write data
			_spiEeprom.writeStream(address, bytes, 2);
		}

		// Read a grid cell from the RAM (only informations for the BF Algorithm)
		void getGridCell(BFSValue* bfsValue, MapCoordinate coor)
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
			_spiEeprom.readStream(address, bytes, 2);

			bfsValue->solverState = bytes[0];
			bfsValue->id = bytes[1];
		}

		// Set a grid cell in the RAM (including informations for the BF Algorithm)
		void setGridCell(GridCell gridCell, BFSValue bfsValue, MapCoordinate coor)
		{
			// Memory address
			uint32_t address;

			// Calculate address
			address = ((coor.x + 0x20) & 0x3f) << 3;	// Bit 4 - 9 = x-Axis / 0 = 0x20
			address += ((coor.y + 0x20) & 0x3f) << 9;	// Bit 10 - 15 = y-Axis / 0 = 0x20
			address += ((coor.floor & 0x1) << 15);		// MSB (16.Bit) = floor

			// Data as a byte array
			uint8_t bytes[4] = { gridCell.cellConnections, gridCell.cellState, bfsValue.solverState, bfsValue.id };

			// Write data
			_spiEeprom.writeStream(address, bytes, 4);
		}

		// Read a grid cell from the RAM (includeing informations for the BF Algorithm)
		void getGridCell(GridCell* gridCell, BFSValue* bfsValue, MapCoordinate coor)
		{
			// Memory address
			uint32_t address;

			// Calculate address
			address = ((coor.x + 0x20) & 0x3f) << 3;	// Bit 4 - 9 = x-Axis / 0 = 0x20
			address += ((coor.y + 0x20) & 0x3f) << 9;	// Bit 10 - 15 = y-Axis / 0 = 0x20
			address += ((coor.floor & 0x1) << 15);		// MSB (16.Bit) = floor

			// Data as a byte array
			uint8_t bytes[4];

			// Read data
			_spiEeprom.readStream(address, bytes, 4);

			// Return data
			gridCell->cellConnections = bytes[0];
			gridCell->cellState = bytes[1];
			bfsValue->solverState = bytes[2];
			bfsValue->id = bytes[3];
		}
		
		namespace BFAlgorithm
		{
			// Find the shortest known path from a to b
			ReturnCode findShortestPath(MapCoordinate start, uint8_t* directions, uint8_t maxPathLength, bool(*goalCondition)(MapCoordinate coor, GridCell cell))
			{
				static uint8_t currentID = 0;

				StaticQueue<MapCoordinate, 64> queue; // MaxSize = 64, because this is the maximum needed size for an 64x64 empty grid (worst case scenario)

				GridCell gridCellV;
				BFSValue bfsValueV;
				MapCoordinate coorV;

				GridCell gridCellW;
				BFSValue bfsValueW;
				MapCoordinate coorW;

				uint8_t distance = 0;

				currentID += 1; // Change ID every time

				setGridCell(BFSValue){ SolverState::discovered, currentID }, start);

				if (queue.enqueue(start) != ReturnCode::ok)
				{
					return ReturnCode::error;
				}

				while (!queue.isEmpty())
				{
					if (queue.dequeue(&coorV) != ReturnCode::ok)
					{
						return ReturnCode::error;
					}

					if (goalCondition(coorV, gridCellV))
					{
						// Go the whole way backwards...
						while (coorV != start)
						{
							getGridCell(&bfsValueV, coorV);

							switch (bfsValueV.solverState)
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
								return ReturnCode::error;
							}

							if (distance > maxPathLength)
							{
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
						if (coorV.y < maxY && (gridCellV.cellConnections & (uint8_t)Direction::north))
						{
							coorW = { coorV.x, coorV.y + 1, coorV.floor };
							getGridCell(&gridCellW, &bfsValueW, coorW);

							if (!((uint8_t)gridCellV.cellState & (uint8_t)CellState::blackTile))
							{
								if ((bfsValueW.id == currentID) && !((uint8_t)bfsValueW.solverState & (uint8_t)SolverState::discovered))
								{
									bfsValueW = { (SolverState)((uint8_t)SolverState::discovered | (uint8_t)SolverState::south), currentID }; // Set discovered-bit, store the shortest path back and store the ID


									if (queue.enqueue(coorW) != ReturnCode::ok)
									{
										return ReturnCode::error;
									}

									setGridCell(bfsValueW, coorW);
								}
							}
						}

						// Check the east
						if (coorV.x < maxX && (gridCellV.cellConnections & (uint8_t)Direction::east))
						{
							coorW = { coorV.x + 1, coorV.y, coorV.floor };
							getGridCell(&gridCellW, &bfsValueW, coorW);

							if (!((uint8_t)gridCellV.cellState & (uint8_t)CellState::blackTile))
							{
								if ((bfsValueW.id == currentID) && !((uint8_t)bfsValueW.solverState & (uint8_t)SolverState::discovered))
								{
									bfsValueW = { (SolverState)((uint8_t)SolverState::discovered | (uint8_t)SolverState::west), currentID }; // Set discovered-bit, store the shortest path back and store the ID


									if (queue.enqueue(coorW) != ReturnCode::ok)
									{
										return ReturnCode::error;
									}

									setGridCell(bfsValueW, coorW);
								}
							}
						}

						// Check the south
						if (coorV.y > minY && (gridCellV.cellConnections & (uint8_t)Direction::south))
						{
							coorW = { coorV.x, coorV.y - 1, coorV.floor };
							getGridCell(&gridCellW, &bfsValueW, coorW);

							if (!((uint8_t)gridCellV.cellState & (uint8_t)CellState::blackTile))
							{
								if ((bfsValueW.id == currentID) && !((uint8_t)bfsValueW.solverState & (uint8_t)SolverState::discovered))
								{
									bfsValueW = { (SolverState)((uint8_t)SolverState::discovered | (uint8_t)SolverState::north), currentID }; // Set discovered-bit, store the shortest path back and store the ID


									if (queue.enqueue(coorW) != ReturnCode::ok)
									{
										return ReturnCode::error;
									}

									setGridCell(bfsValueW, coorW);
								}
							}
						}

						// Check the west
						if (coorV.x > minX && (gridCellV.cellConnections & (uint8_t)Direction::west))
						{
							coorW = { coorV.x - 1, coorV.y, coorV.floor };
							getGridCell(&gridCellW, &bfsValueW, coorW);

							if (!((uint8_t)gridCellV.cellState & (uint8_t)CellState::blackTile))
							{
								if ((bfsValueW.id == currentID) && !((uint8_t)bfsValueW.solverState & (uint8_t)SolverState::discovered))
								{
									bfsValueW = { (SolverState)((uint8_t)SolverState::discovered | (uint8_t)SolverState::east), currentID }; // Set discovered-bit, store the shortest path back and store the ID


									if (queue.enqueue(coorW) != ReturnCode::ok)
									{
										return ReturnCode::error;
									}

									setGridCell(bfsValueW, coorW);
								}
							}
						}
					}
				}

				return ReturnCode::error;
			}
		}
	}
}
