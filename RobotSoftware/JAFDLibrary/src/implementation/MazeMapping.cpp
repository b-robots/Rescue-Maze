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
			// SS Pin of RAM
			uint8_t _ramSSPin;

			// Class for handling the EEPROM
			SpiEeprom::Eeprom25LC1024 _spiEeprom(2);
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

		// Comparison operators for GridCell
		inline bool operator==(const GridCell& lhs, const GridCell& rhs) { return (lhs.cellConnections == rhs.cellConnections && lhs.cellState == rhs.cellState); }
		inline bool operator!=(const GridCell& lhs, const GridCell& rhs) { return !(lhs == rhs); }

		// Comparison operators for MapCoordinate
		inline bool operator==(const MapCoordinate& lhs, const MapCoordinate& rhs) { return (lhs.floor == rhs.floor && lhs.x == rhs.x && lhs.y == rhs.y); }
		inline bool operator!=(const MapCoordinate& lhs, const MapCoordinate& rhs) { return !(lhs == rhs); }

		// Get the position of the Ramp
		RampDirection getRampDirection(GridCell cell)
		{
			return (RampDirection)((cell.cellState >> 4) & 0x3);
		}

		// Setup the MazeMapper
		ReturnCode mazeMapperSetup(MazeMapperSet settings)
		{
			_ramSSPin = settings.ramSSPin;
			_spiEeprom = SpiEeprom::Eeprom25LC1024(_ramSSPin);

			return ReturnCode::ok;
		}

		// Set a grid cell in the RAM
		ReturnCode setGridCell(GridCell gridCell, MapCoordinate coor)
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

			return ReturnCode::ok;
		}

		// Read a grid cell from the RAM
		ReturnCode getGridCell(GridCell* gridCell, MapCoordinate coor)
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

			return ReturnCode::ok;
		}

		// Set a grid cell in the RAM (only informations for the BF Algorithm)
		ReturnCode setGridCell(BFAlgorithm::SolverValue solverValue, MapCoordinate coor)
		{
			// Memory address
			uint32_t address;
			
			// Calculate address
			address = ((coor.x + 0x20) & 0x3f) << 3;	// Bit 4 - 9 = x-Axis / 0 = 0x20
			address += ((coor.y + 0x20) & 0x3f) << 9;	// Bit 10 - 15 = y-Axis / 0 = 0x20
			address += ((coor.floor & 0x1) << 15);		// MSB (16.Bit) = floor
			address += 2;								// Go to the solver value

			// Write data
			_spiEeprom.writeByte(address, (uint8_t)solverValue);

			return ReturnCode::ok;
		}

		// Read a grid cell from the RAM (only informations for the BF Algorithm)
		ReturnCode getGridCell(BFAlgorithm::SolverValue* solverValue, MapCoordinate coor)
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
			*solverValue = (BFAlgorithm::SolverValue)_spiEeprom.readByte(address);

			return ReturnCode::ok;
		}

		// Set a grid cell in the RAM (including informations for the BF Algorithm)
		ReturnCode setGridCell(GridCell gridCell, BFAlgorithm::SolverValue solverValue, MapCoordinate coor)
		{
			// Memory address
			uint32_t address;

			// Calculate address
			address = ((coor.x + 0x20) & 0x3f) << 3;	// Bit 4 - 9 = x-Axis / 0 = 0x20
			address += ((coor.y + 0x20) & 0x3f) << 9;	// Bit 10 - 15 = y-Axis / 0 = 0x20
			address += ((coor.floor & 0x1) << 15);		// MSB (16.Bit) = floor

			// Data as a byte array
			uint8_t bytes[3] = { gridCell.cellConnections, gridCell.cellState, (uint8_t)solverValue };

			// Write data
			_spiEeprom.writeStream(address, bytes, 3);

			return ReturnCode::ok;
		}

		// Read a grid cell from the RAM (includeing informations for the BF Algorithm)
		ReturnCode getGridCell(GridCell* gridCell, BFAlgorithm::SolverValue* solverValue, MapCoordinate coor)
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
			*solverValue = (BFAlgorithm::SolverValue)bytes[2];

			return ReturnCode::ok;
		}

		namespace BFAlgorithm
		{
			// Clear all Solver-Values in the EEPROM
			void clearSolverValues(uint8_t floor)
			{
				for (int8_t x = minX; x <= maxX; x++)
				{
					for (int8_t y = minY; x <= maxY; y++)
					{
						setGridCell((SolverValue)0, { x, y, floor});
					}
				}
			}

			// Find the shortest known path from a to b
			ReturnCode findShortestPath(MapCoordinate start, Direction* directions, uint8_t maxPathLength, bool(*goalCondition)(MapCoordinate coor, GridCell cell))
			{
				StaticQueue<MapCoordinate, 64> queue; // MaxSize = 64, because this is the maximum needed size for an 64x64 empty grid (worst case scenario)

				GridCell gridCellV;
				SolverValue solverValueV;
				MapCoordinate coorV;

				GridCell gridCellW;
				SolverValue solverValueW;
				MapCoordinate coorW;

				uint8_t distance = 0;

				setGridCell(SolverValue::discovered, start);
				queue.enqueue(start);

				while (!queue.isEmpty())
				{
					queue.dequeue(&coorV);

					if (goalCondition(coorV, gridCellV))
					{
						// Go the whole way backwards...
						while (coorV != start)
						{
							getGridCell(&solverValueV, coorV);

							switch ((SolverValue)((uint8_t)solverValueV & 0x3))
							{
							case SolverValue::north:
								directions[distance++] = Direction::south; // Set the opposite direction
								coorV = { coorV.x, coorV.y + 1, coorV.floor };
								break;

							case SolverValue::east:
								directions[distance++] = Direction::west; // Set the opposite direction
								coorV = { coorV.x + 1, coorV.y, coorV.floor };
								break;

							case SolverValue::south:
								directions[distance++] = Direction::north; // Set the opposite direction
								coorV = { coorV.x, coorV.y - 1, coorV.floor };
								break;

							case SolverValue::west:
								directions[distance++] = Direction::east; // Set the opposite direction
								coorV = { coorV.x - 1, coorV.y, coorV.floor };
								break;

							default:
								clearSolverValues(start.floor);
								return ReturnCode::error;
							}

							if (distance > maxPathLength)
							{
								clearSolverValues(start.floor);
								return ReturnCode::aborted;
							}
						}

						std::reverse(directions, directions + distance - 1);

						clearSolverValues(start.floor);
						return ReturnCode::ok;
					}
					else
					{
						getGridCell(&gridCellV, coorV);

						// Check the north
						if (coorV.y < maxY && (gridCellV.cellConnections & (uint8_t)Direction::north))
						{
							coorW = { coorV.x, coorV.y + 1, coorV.floor };
							getGridCell(&gridCellW, &solverValueW, coorW);

							if (!(gridCellV.cellState & (uint8_t)CellState::blackTile))
							{
								if (!((uint8_t)solverValueW & (uint8_t)SolverValue::discovered))
								{
									solverValueW = (SolverValue)((uint8_t)SolverValue::discovered | (uint8_t)SolverValue::south); // Set discovered-bit and store the shortest path back

									queue.enqueue(coorW);

									setGridCell(solverValueW, coorW);
								}
							}
						}

						// Check the east
						if (coorV.x < maxX && (gridCellV.cellConnections & (uint8_t)Direction::east))
						{
							coorW = { coorV.x + 1, coorV.y, coorV.floor };
							getGridCell(&gridCellW, &solverValueW, coorW);

							if (!(gridCellV.cellState & (uint8_t)CellState::blackTile))
							{
								if (!((uint8_t)solverValueW & (uint8_t)SolverValue::discovered))
								{
									solverValueW = (SolverValue)((uint8_t)SolverValue::discovered | (uint8_t)SolverValue::west); // Set discovered-bit and store the shortest path back

									queue.enqueue(coorW);

									setGridCell(solverValueW, coorW);
								}
							}
						}

						// Check the south
						if (coorV.y > minY && (gridCellV.cellConnections & (uint8_t)Direction::south))
						{
							coorW = { coorV.x, coorV.y - 1, coorV.floor };
							getGridCell(&gridCellW, &solverValueW, coorW);

							if (!(gridCellV.cellState & (uint8_t)CellState::blackTile))
							{
								if (!((uint8_t)solverValueW & (uint8_t)SolverValue::discovered))
								{
									solverValueW = (SolverValue)((uint8_t)SolverValue::discovered | (uint8_t)SolverValue::east); // Set discovered-bit and store the shortest path back

									queue.enqueue(coorW);

									setGridCell(solverValueW, coorW);
								}
							}
						}

						// Check the west
						if (coorV.x > minX && (gridCellV.cellConnections & (uint8_t)Direction::west))
						{
							coorW = { coorV.x - 1, coorV.y, coorV.floor };
							getGridCell(&gridCellW, &solverValueW, coorW);

							if (!(gridCellV.cellState & (uint8_t)CellState::blackTile))
							{
								if (!((uint8_t)solverValueW & (uint8_t)SolverValue::discovered))
								{
									solverValueW = (SolverValue)((uint8_t)SolverValue::discovered | (uint8_t)SolverValue::north); // Set discovered-bit and store the shortest path back

									queue.enqueue(coorW);

									setGridCell(solverValueW, coorW);
								}
							}
						}
					}
				}

				clearSolverValues(start.floor);
				return ReturnCode::error;
			}
		}
	}
}
