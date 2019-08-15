/*
This part of the Library is responsible for mapping the maze and finding the shortest paths.
*/

#include "MazeMapping.h"

namespace JAFTD
{
	namespace MazeMapping
	{
		static const MapCoordinate homePosition = { 0, 0, 0 };

		// Setup the MazeMapper
		ReturnCode mazeMapperSetup(MazeMapperSet settings)
		{
			ramSSPin = settings.ramSSPin;
			spiRam = SpiRAM(0, ramSSPin);

			return ReturnCode::ok;
		}

		// Set a grid cell in the RAM
		ReturnCode setGridCell(GridCell gridCell, MapCoordinate coor)
		{
			// Memory address
			uint16_t address;;

			// Calculate address
			address = ((coor.x + 0b10000) & 0b00111111) << 2; // Bit 3 - 8 = x-Axis / 0 = 0b10000
			address += ((coor.y + 0b10000) & 0b00111111) << 8; // Bit 9 - 14 = y-Axis / 0 = 0b100000
			address += ((coor.floor & 0b00000001) << 14); // MSB (15.Bit) = floor

			// Data as a byte array
			char bytes[4] = { (char)gridCell.cellConnections, (char)gridCell.cellState, 0, 0 };

			// Write data
			spiRam.write_stream(address, bytes, 4);

			return ReturnCode::ok;
		}

		// Read a grid cell from the RAM
		ReturnCode getGridCell(GridCell* gridCell, MapCoordinate coor)
		{
			// Memory address
			uint16_t address;;

			// Calculate address
			address = ((coor.x + 0b10000) & 0b00111111) << 2; // Bit 3 - 8 = x-Axis / 0 = 0b10000
			address += ((coor.y + 0b10000) & 0b00111111) << 8; // Bit 9 - 14 = y-Axis / 0 = 0b100000
			address += ((coor.floor & 0b00000001) << 14); // MSB (15.Bit) = floor

			// Data as a byte array
			char bytes[4];

			// Read data
			spiRam.write_stream(address, bytes, 4);

			// Return data
			gridCell->cellConnections = bytes[0];
			gridCell->cellState = bytes[1];

			return ReturnCode::ok;
			return ReturnCode::ok;
		}

		// Set a grid cell in the RAM (only informations for the maze solver)
		ReturnCode setGridCell(uint16_t solverValue, MapCoordinate coor)
		{
			// Memory address
			uint16_t address;;

			// Calculate address
			address = ((coor.x + 0b10000) & 0b00111111) << 2; // Bit 3 - 8 = x-Axis / 0 = 0b10000
			address += ((coor.y + 0b10000) & 0b00111111) << 8; // Bit 9 - 14 = y-Axis / 0 = 0b100000
			address += ((coor.floor & 0b00000001) << 14); // MSB (15.Bit) = floor
			address += 2; // Go to the solver value

			// Data as a byte array
			char bytes[2] = { solverValue & 0b0000000011111111, solverValue >> 8 };

			// Write data
			spiRam.write_stream(address, bytes, 2);

			return ReturnCode::ok;
		}

		// Read a grid cell from the RAM (only informations for the maze solver)
		ReturnCode getGridCell(uint16_t* solverValue, MapCoordinate coor)
		{
			// Memory address
			uint16_t address;;

			// Calculate address
			address = ((coor.x + 0b10000) & 0b00111111) << 2; // Bit 3 - 8 = x-Axis / 0 = 0b10000
			address += ((coor.y + 0b10000) & 0b00111111) << 8; // Bit 9 - 14 = y-Axis / 0 = 0b100000
			address += ((coor.floor & 0b00000001) << 14); // MSB (15.Bit) = floor
			address += 2; // Go to the solver value

			// Data as a byte array
			char bytes[2];

			// Read data
			spiRam.write_stream(address, bytes, 4);

			// Return data
			*solverValue = bytes[2] + bytes[3] << 8;

			return ReturnCode::ok;
		}

		// Set a grid cell in the RAM (including informations for the maze solver)
		ReturnCode setGridCell(GridCell gridCell, uint16_t solverValue, MapCoordinate coor)
		{
			// Memory address
			uint16_t address;;

			// Calculate address
			address = ((coor.x + 0b10000) & 0b00111111) << 2; // Bit 3 - 8 = x-Axis / 0 = 0b10000
			address += ((coor.y + 0b10000) & 0b00111111) << 8; // Bit 9 - 14 = y-Axis / 0 = 0b100000
			address += ((coor.floor & 0b00000001) << 14); // MSB (15.Bit) = floor

			// Data as a byte array
			char bytes[4] = { gridCell.cellConnections, gridCell.cellState, solverValue & 0b0000000011111111, solverValue >> 8 };

			// Write data
			spiRam.write_stream(address, bytes, 2);

			return ReturnCode::ok;
		}

		// Read a grid cell from the RAM (includeing informations for the maze solver)
		ReturnCode getGridCell(GridCell* gridCell, uint16_t* solverValue, MapCoordinate coor)
		{
			// Memory address
			uint16_t address;;

			// Calculate address
			address = ((coor.x + 0b10000) & 0b00111111) << 2; // Bit 3 - 8 = x-Axis / 0 = 0b10000
			address += ((coor.y + 0b10000) & 0b00111111) << 8; // Bit 9 - 14 = y-Axis / 0 = 0b100000
			address += ((coor.floor & 0b00000001) << 14); // MSB (15.Bit) = floor

			// Data as a byte array
			char bytes[4];

			// Read data
			spiRam.write_stream(address, bytes, 4);

			// Return data
			gridCell->cellConnections = bytes[0];
			gridCell->cellState = bytes[1];
			*solverValue = bytes[2] + bytes[3] << 8;

			return ReturnCode::ok;
		}

		namespace BFAlgorithm
		{
			// Find the shortest known path from a to b
			ReturnCode findShortestPath(MapCoordinate a, MapCoordinate goal, Direction* directions, uint8_t maxPathLength)
			{
				return ReturnCode::ok;
			}
		}
	}
}
