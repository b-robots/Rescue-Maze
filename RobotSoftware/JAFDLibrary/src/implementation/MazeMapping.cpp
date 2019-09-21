/*
This part of the Library is responsible for mapping the maze and finding the shortest paths.
*/

#include "MazeMapping_private.h"

namespace JAFD
{
	namespace MazeMapping
	{
		// Anonymous namespace = private variables & methods
		namespace
		{
			// SS Pin of RAM
			byte ramSSPin;

			// TODO: Rewrite the SPIRam Library, it causes a crash!!
			// Class for handling the RAM
			//SpiRAM spiRam(0, 0);
		}

		// Home Position
		const MapCoordinate homePosition = { 0, 0, 0 };

		// Comparison operators for GridCell
		inline bool operator==(const GridCell& lhs, const GridCell& rhs) { return (lhs.cellConnections == rhs.cellConnections && lhs.cellState == rhs.cellState); }
		inline bool operator!=(const GridCell& lhs, const GridCell& rhs) { return !(lhs == rhs); }

		// Comparison operators for MapCoordinate
		inline bool operator==(const MapCoordinate& lhs, const MapCoordinate& rhs) { return (lhs.floor == rhs.floor && lhs.x == rhs.x && lhs.y == rhs.y); }
		inline bool operator!=(const MapCoordinate& lhs, const MapCoordinate& rhs) { return !(lhs == rhs); }

		// Setup the MazeMapper
		ReturnCode mazeMapperSetup(MazeMapperSet settings)
		{
			ramSSPin = settings.ramSSPin;
			//spiRam = SpiRAM(0, ramSSPin);

			return ReturnCode::ok;
		}

		// Set a grid cell in the RAM
		ReturnCode setGridCell(GridCell gridCell, MapCoordinate coor)
		{
			// Memory address
			uint16_t address;;

			// Calculate address
			address = ((coor.x + 0b100000) & 0b00111111) << 2;	// Bit 3 - 8 = x-Axis / 0 = 0b100000
			address += ((coor.y + 0b100000) & 0b00111111) << 8; // Bit 9 - 14 = y-Axis / 0 = 0b100000
			address += ((coor.floor & 0b00000001) << 14);		// MSB (15.Bit) = floor

			// Data as a byte array
			char bytes[2] = { (char)gridCell.cellConnections, (char)gridCell.cellState };

			// Write data
			//spiRam.write_stream(address, bytes, 2);

			return ReturnCode::ok;
		}

		// Read a grid cell from the RAM
		ReturnCode getGridCell(GridCell* gridCell, MapCoordinate coor)
		{
			// Memory address
			uint16_t address;;

			// Calculate address
			address = ((coor.x + 0b100000) & 0b00111111) << 2;	// Bit 3 - 8 = x-Axis / 0 = 0b100000
			address += ((coor.y + 0b100000) & 0b00111111) << 8; // Bit 9 - 14 = y-Axis / 0 = 0b100000
			address += ((coor.floor & 0b00000001) << 14);		// MSB (15.Bit) = floor

			// Data as a byte array
			char bytes[2];

			// Read data
			//spiRam.write_stream(address, bytes, 2);

			// Return data
			gridCell->cellConnections = bytes[0];
			gridCell->cellState = bytes[1];

			return ReturnCode::ok;
		}

		// Set a grid cell in the RAM (only informations for the BF Algorithm)
		ReturnCode setGridCell(uint16_t bfValue, MapCoordinate coor)
		{
			// Memory address
			uint16_t address;;

			// Calculate address
			address = ((coor.x + 0b100000) & 0b00111111) << 2;	// Bit 3 - 8 = x-Axis / 0 = 0b100000
			address += ((coor.y + 0b100000) & 0b00111111) << 8; // Bit 9 - 14 = y-Axis / 0 = 0b100000
			address += ((coor.floor & 0b00000001) << 14);		// MSB (15.Bit) = floor
			address += 2;										// Go to the solver value

			// Data as a byte array
			char bytes[2] = { bfValue & 0b0000000011111111, bfValue >> 8 };

			// Write data
			//spiRam.write_stream(address, bytes, 2);

			return ReturnCode::ok;
		}

		// Read a grid cell from the RAM (only informations for the BF Algorithm)
		ReturnCode getGridCell(uint16_t* bfValue, MapCoordinate coor)
		{
			// Memory address
			uint16_t address;;

			// Calculate address
			address = ((coor.x + 0b100000) & 0b00111111) << 2;	// Bit 3 - 8 = x-Axis / 0 = 0b100000
			address += ((coor.y + 0b100000) & 0b00111111) << 8; // Bit 9 - 14 = y-Axis / 0 = 0b100000
			address += ((coor.floor & 0b00000001) << 14);		// MSB (15.Bit) = floor
			address += 2;										// Go to the solver value

			// Data as a byte array
			char bytes[2];

			// Read data
			//spiRam.write_stream(address, bytes, 4);

			// Return data
			*bfValue = bytes[2] + bytes[3] << 8;

			return ReturnCode::ok;
		}

		// Set a grid cell in the RAM (including informations for the BF Algorithm)
		ReturnCode setGridCell(GridCell gridCell, uint16_t bfValue, MapCoordinate coor)
		{
			// Memory address
			uint16_t address;;

			// Calculate address
			address = ((coor.x + 0b100000) & 0b00111111) << 2;	// Bit 3 - 8 = x-Axis / 0 = 0b100000
			address += ((coor.y + 0b100000) & 0b00111111) << 8; // Bit 9 - 14 = y-Axis / 0 = 0b100000
			address += ((coor.floor & 0b00000001) << 14);		// MSB (15.Bit) = floor

			// Data as a byte array
			char bytes[4] = { gridCell.cellConnections, gridCell.cellState, bfValue & 0b0000000011111111, bfValue >> 8 };

			// Write data
			//spiRam.write_stream(address, bytes, 2);

			return ReturnCode::ok;
		}

		// Read a grid cell from the RAM (includeing informations for the BF Algorithm)
		ReturnCode getGridCell(GridCell* gridCell, uint16_t* bfValue, MapCoordinate coor)
		{
			// Memory address
			uint16_t address;;

			// Calculate address
			address = ((coor.x + 0b100000) & 0b00111111) << 2;	// Bit 3 - 8 = x-Axis / 0 = 0b100000
			address += ((coor.y + 0b100000) & 0b00111111) << 8; // Bit 9 - 14 = y-Axis / 0 = 0b100000
			address += ((coor.floor & 0b00000001) << 14);		// MSB (15.Bit) = floor

			// Data as a byte array
			char bytes[4];

			// Read data
			//spiRam.write_stream(address, bytes, 4);

			// Return data
			gridCell->cellConnections = bytes[0];
			gridCell->cellState = bytes[1];
			*bfValue = bytes[2] + bytes[3] << 8;

			return ReturnCode::ok;
		}

		namespace BFAlgorithm
		{
			auto equalsFive = [](int a) noexcept -> bool
			{
				return a == 5;
			};

			// Find the shortest known path from a to b
			ReturnCode findShortestPath(MapCoordinate a, Direction* directions, uint8_t maxPathLength, bool(*goalCondition)(MapCoordinate coor, GridCell cell))
			{
				StaticQueue<MapCoordinate, 64> queue; // MaxSize = 64, because this is the maximum needed size for an 64x64 empty grid (worst case scenario)

				GridCell gridCellV;
				uint16_t bfValueV;
				GridCell gridCellW;
				uint16_t bfValueW;
				MapCoordinate v;
				MapCoordinate w;
				uint8_t distance = 0;
				Direction shortestDirection;
				bool foundWay;

				setGridCell(1 << 15, a); // 1<<15 means discovered (and the first 15 Bits are zero, thsi means it is the start node)
				queue.enqueue(a);

				while (!queue.isEmpty())
				{
					queue.dequeue(&v);

					if (goalCondition(v, gridCellV))
					{
						// Go the whole way backwards...
						while (v != a)
						{
							getGridCell(&gridCellV, &bfValueV, v);

							foundWay = false;

							if (v.y < MAX_Y && (gridCellV.cellConnections & Direction::north)) // Check the north
							{
								w = { v.x, v.y + 1, v.floor };
								getGridCell(&gridCellW, &bfValueW, w);

								if ((bfValueW & ~(1 << 15)) == --distance)
								{
									// The north is the shortest path!
									directions[distance] == Direction::south; // From the other direction it's the south
									foundWay = true;
								}
							}
							else if (!foundWay && v.x < MAX_X && (gridCellV.cellConnections & Direction::east))	// Check the east
							{
								w = { v.x + 1, v.y, v.floor };
								getGridCell(&gridCellW, &bfValueW, w);

								if ((bfValueW & ~(1 << 15)) == --distance)
								{
									// The north is the shortest path!
									directions[distance] == Direction::west; // From the other direction it's the west
									foundWay = true;
								}
							}
							else if (!foundWay && v.y > MIN_Y && (gridCellV.cellConnections & Direction::south)) // Check the south
							{
								w = { v.x, v.y - 1, v.floor };
								getGridCell(&gridCellW, &bfValueW, w);

								if ((bfValueW & ~(1 << 15)) == --distance)
								{
									// The north is the shortest path!
									directions[distance] == Direction::north; // From the other direction it's the north
									foundWay = true;
								}
							}
							else if (!foundWay && v.x > MIN_X && (gridCellV.cellConnections & Direction::west)) // Check the west
							{
								w = { v.x - 1, v.y, v.floor };
								getGridCell(&gridCellW, &bfValueW, w);

								if ((bfValueW & ~(1 << 15)) == --distance)
								{
									// The north is the shortest path!
									directions[distance] == Direction::east; // From the other direction it's the east
									foundWay = true;
								}
							}
						}

						return ReturnCode::ok;
					}
					else
					{
						getGridCell(&gridCellV, v);

						distance++;

						if (distance > maxPathLength) return ReturnCode::error;

						// Check the north
						if (v.y < MAX_Y && (gridCellV.cellConnections & Direction::north))
						{
							w = { v.x, v.y + 1, v.floor };
							getGridCell(&gridCellW, &bfValueW, w);

							if (!(bfValueW >> 15))
							{
								bfValueW = 1 << 15 + distance;

								queue.enqueue(w);

								setGridCell(bfValueW, w);
							}
						}

						// Check the east
						if (v.x < MAX_X && (gridCellV.cellConnections & Direction::east))
						{
							w = { v.x + 1, v.y, v.floor };
							getGridCell(&gridCellW, &bfValueW, w);

							if (!(bfValueW >> 15))
							{
								bfValueW = 1 << 15 + distance;

								queue.enqueue(w);

								setGridCell(bfValueW, w);
							}
						}

						// Check the south
						if (v.y > MIN_Y && (gridCellV.cellConnections & Direction::south))
						{
							w = { v.x, v.y - 1, v.floor };
							getGridCell(&gridCellW, &bfValueW, w);

							if (!(bfValueW >> 15))
							{
								bfValueW = 1 << 15 + distance;

								queue.enqueue(w);

								setGridCell(bfValueW, w);
							}
						}

						// Check the west
						if (v.x > MIN_X && (gridCellV.cellConnections & Direction::west))
						{
							w = { v.x - 1, v.y, v.floor };
							getGridCell(&gridCellW, &bfValueW, w);

							if (!(bfValueW >> 15))
							{
								bfValueW = 1 << 15 + distance;

								queue.enqueue(w);

								setGridCell(bfValueW, w);
							}
						}
					}
				}

				return ReturnCode::error;
			}
		}
	}
}
