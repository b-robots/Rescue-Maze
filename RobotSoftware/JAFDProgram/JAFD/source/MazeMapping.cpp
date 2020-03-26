/*
This part of the Library is responsible for mapping the maze and finding the shortest paths.
*/

#include "../header/MazeMapping.h"
#include "../header/SpiNVSRAM.h"
#include "../header/StaticQueue.h"
#include "../header/DistanceSensors.h"
#include "../header/SensorFusion.h"
#include "../header/SmoothDriving.h"

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
								directions[distance] = Directions::south; // Set the opposite direction
								coorV = { coorV.x, coorV.y + 1, coorV.floor };
								break;

							case SolverState::east:
								directions[distance] = Directions::west; // Set the opposite direction
								coorV = { coorV.x + 1, coorV.y, coorV.floor };
								break;

							case SolverState::south:
								directions[distance] = Directions::north; // Set the opposite direction
								coorV = { coorV.x, coorV.y - 1, coorV.floor };
								break;

							case SolverState::west:
								directions[distance] = Directions::east; // Set the opposite direction
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
						if (coorV.y < maxY && (gridCellV.cellConnections & Directions::north))
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
						if (coorV.x < maxX && (gridCellV.cellConnections & Directions::east))
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
						if (coorV.y > minY && (gridCellV.cellConnections & Directions::south))
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
						if (coorV.x > minX && (gridCellV.cellConnections & Directions::west))
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
			static MapCoordinate lastPosition = homePosition;
			static Distances lastDistances;
			static DistSensorStates lastDistSensStates;

			Distances currentDistances = SensorFusion::getFusedData().distances;
			DistSensorStates currentDistSensStates = SensorFusion::getFusedData().distSensorState;
			FusedData fusedData = SensorFusion::getFusedData();

			float tempCertainty;
			GridCell tempCell;

			int8_t lfDistSensorEdgeDetect = 0;	// 0 = no edge; 1 = long to short distance edge; -1 = short to long distance edge
			int8_t lbDistSensorEdgeDetect = 0;	// 0 = no edge; 1 = long to short distance edge; -1 = short to long distance edge
			int8_t rfDistSensorEdgeDetect = 0;	// 0 = no edge; 1 = long to short distance edge; -1 = short to long distance edge
			int8_t rbDistSensorEdgeDetect = 0;	// 0 = no edge; 1 = long to short distance edge; -1 = short to long distance edge

			if ((lastDistSensStates.leftFront == DistSensorStatus::overflow && currentDistSensStates.leftFront == DistSensorStatus::ok) ||
				(lastDistSensStates.leftFront == DistSensorStatus::overflow && currentDistSensStates.leftFront == DistSensorStatus::underflow) ||
				(lastDistSensStates.leftFront == DistSensorStatus::ok && currentDistSensStates.leftFront == DistSensorStatus::ok && ((int32_t)currentDistances.leftFront - (int32_t)lastDistances.leftFront) < (int32_t)-JAFDSettings::SensorFusion::minDeltaDistForEdge))
			{
				lfDistSensorEdgeDetect = 1;
			}
			else if ((lastDistSensStates.leftFront == DistSensorStatus::underflow && currentDistSensStates.leftFront == DistSensorStatus::ok) ||
					 (lastDistSensStates.leftFront == DistSensorStatus::underflow && currentDistSensStates.leftFront == DistSensorStatus::overflow) ||
					 (lastDistSensStates.leftFront == DistSensorStatus::ok && currentDistSensStates.leftFront == DistSensorStatus::ok && ((int32_t)currentDistances.leftFront - (int32_t)lastDistances.leftFront) > JAFDSettings::SensorFusion::minDeltaDistForEdge))
			{
				lfDistSensorEdgeDetect = -1;
			}

			if ((lastDistSensStates.leftBack == DistSensorStatus::overflow && currentDistSensStates.leftBack == DistSensorStatus::ok) ||
				(lastDistSensStates.leftBack == DistSensorStatus::overflow && currentDistSensStates.leftBack == DistSensorStatus::underflow) ||
				(lastDistSensStates.leftBack == DistSensorStatus::ok && currentDistSensStates.leftBack == DistSensorStatus::ok && ((int32_t)currentDistances.leftBack - (int32_t)lastDistances.leftBack) < (int32_t)-JAFDSettings::SensorFusion::minDeltaDistForEdge))
			{
				lbDistSensorEdgeDetect = 1;
			}
			else if ((lastDistSensStates.leftBack == DistSensorStatus::underflow && currentDistSensStates.leftBack == DistSensorStatus::ok) ||
					 (lastDistSensStates.leftBack == DistSensorStatus::underflow && currentDistSensStates.leftBack == DistSensorStatus::overflow) ||
					 (lastDistSensStates.leftBack == DistSensorStatus::ok && currentDistSensStates.leftBack == DistSensorStatus::ok && ((int32_t)currentDistances.leftBack - (int32_t)lastDistances.leftBack) > JAFDSettings::SensorFusion::minDeltaDistForEdge))
			{
				lbDistSensorEdgeDetect = -1;
			}

			if ((lastDistSensStates.rightFront == DistSensorStatus::overflow && currentDistSensStates.rightFront == DistSensorStatus::ok) ||
				(lastDistSensStates.rightFront == DistSensorStatus::overflow && currentDistSensStates.rightFront == DistSensorStatus::underflow) ||
				(lastDistSensStates.rightFront == DistSensorStatus::ok && currentDistSensStates.rightFront == DistSensorStatus::ok && ((int32_t)currentDistances.rightFront - (int32_t)lastDistances.rightFront) < (int32_t)-JAFDSettings::SensorFusion::minDeltaDistForEdge))
			{
				rfDistSensorEdgeDetect = 1;
			}
			else if ((lastDistSensStates.rightFront == DistSensorStatus::underflow && currentDistSensStates.rightFront == DistSensorStatus::ok) ||
					 (lastDistSensStates.rightFront == DistSensorStatus::underflow && currentDistSensStates.rightFront == DistSensorStatus::overflow) ||
					 (lastDistSensStates.rightFront == DistSensorStatus::ok && currentDistSensStates.rightFront == DistSensorStatus::ok && ((int32_t)currentDistances.rightFront - (int32_t)lastDistances.rightFront) > JAFDSettings::SensorFusion::minDeltaDistForEdge))
			{
				rfDistSensorEdgeDetect = -1;
			}

			if ((lastDistSensStates.rightBack == DistSensorStatus::overflow && currentDistSensStates.rightBack == DistSensorStatus::ok) ||
				(lastDistSensStates.rightBack == DistSensorStatus::overflow && currentDistSensStates.rightBack == DistSensorStatus::underflow) ||
				(lastDistSensStates.rightBack == DistSensorStatus::ok && currentDistSensStates.rightBack == DistSensorStatus::ok && ((int32_t)currentDistances.rightBack - (int32_t)lastDistances.rightBack) < (int32_t)-JAFDSettings::SensorFusion::minDeltaDistForEdge))
			{
				rbDistSensorEdgeDetect = 1;
			}
			else if ((lastDistSensStates.rightBack == DistSensorStatus::underflow && currentDistSensStates.rightBack == DistSensorStatus::ok) ||
					 (lastDistSensStates.rightBack == DistSensorStatus::underflow && currentDistSensStates.rightBack == DistSensorStatus::overflow) ||
					 (lastDistSensStates.rightBack == DistSensorStatus::ok && currentDistSensStates.rightBack == DistSensorStatus::ok && ((int32_t)currentDistances.rightBack - (int32_t)lastDistances.rightBack) > JAFDSettings::SensorFusion::minDeltaDistForEdge))
			{
				rbDistSensorEdgeDetect = -1;
			}

			if (lastPosition != MapCoordinate(fusedData.robotState.mapCoordinate))
			{
				tempCertainty = 0.0f;
				tempCell.cellConnections = Directions::nowhere;
				tempCell.cellState = 0;
			}
			else
			{
				tempCertainty = certainty;
				tempCell = cell;
			}

			//if (lfDistSensorEdgeDetect) Serial.print("LF ");
			//if (lbDistSensorEdgeDetect) Serial.print("LB ");
			//if (rfDistSensorEdgeDetect) Serial.print("RF ");
			//if (rbDistSensorEdgeDetect) Serial.print("RB ");
			//Serial.println("");

			//if (fabs(fusedData.robotState.rotation.y) < JAFDSettings::SensorFusion::maxPitchForDistSensor)
			//{
			//	if (fusedData.distSensorState.leftFront == DistSensorStatus::ok)
			//	{
			//		bool hitPointIsOk = false;

			//		// Measurement is ok
			//		// Check if resulting hit point is a 90° wall on left side of us
			//		if (fusedData.robotState.heading == AbsoluteDir::north || fusedData.robotState.heading == AbsoluteDir::south)
			//		{
			//			float hitX = sinf(fusedData.robotState.rotation.x) * fusedData.distances.leftFront / 10.0f + fusedData.robotState.position.x + cosf(fusedData.robotState.rotation.x + JAFDSettings::Mechanics::distSensLRAngleToMiddle) * JAFDSettings::Mechanics::distSensLRDistToMiddle;

			//			if (fabs(hitX - fusedData.robotState.mapCoordinate.x * JAFDSettings::Field::cellWidth) < JAFDSettings::MazeMapping::widthSecureDetectFactor * JAFDSettings::Field::cellWidth)
			//			{
			//				hitPointIsOk = true;
			//			}
			//		}
			//		else
			//		{
			//			float hitY = cosf(fusedData.robotState.rotation.x) * fusedData.distances.leftFront / 10.0f + fusedData.robotState.position.y + sinf(fusedData.robotState.rotation.x + JAFDSettings::Mechanics::distSensLRAngleToMiddle) * JAFDSettings::Mechanics::distSensLRDistToMiddle;;

			//			if (fabs(hitY - fusedData.robotState.mapCoordinate.y * JAFDSettings::Field::cellWidth) < JAFDSettings::MazeMapping::widthSecureDetectFactor * JAFDSettings::Field::cellWidth)
			//			{
			//				hitPointIsOk = true;
			//			}
			//		}

			//		if (hitPointIsOk)
			//		{
			//			Serial.println("LF");
			//		}
			//	}

			//	if (fusedData.distSensorState.leftBack == DistSensorStatus::ok)
			//	{
			//		bool hitPointIsOk = false;

			//		// Measurement is ok
			//		// Check if resulting hit point is a 90° wall on left side of us
			//		if (fusedData.robotState.heading == AbsoluteDir::north || fusedData.robotState.heading == AbsoluteDir::south)
			//		{
			//			float hitX = sinf(fusedData.robotState.rotation.x) * fusedData.distances.leftBack / 10.0f + fusedData.robotState.position.x - cosf(fusedData.robotState.rotation.x + JAFDSettings::Mechanics::distSensLRAngleToMiddle) * JAFDSettings::Mechanics::distSensLRDistToMiddle;

			//			if (fabs(hitX - fusedData.robotState.mapCoordinate.x * JAFDSettings::Field::cellWidth) < JAFDSettings::MazeMapping::widthSecureDetectFactor * JAFDSettings::Field::cellWidth)
			//			{
			//				hitPointIsOk = true;
			//			}
			//		}
			//		else
			//		{
			//			float hitY = cosf(fusedData.robotState.rotation.x) * fusedData.distances.leftBack / 10.0f + fusedData.robotState.position.y - sinf(fusedData.robotState.rotation.x + JAFDSettings::Mechanics::distSensLRAngleToMiddle) * JAFDSettings::Mechanics::distSensLRDistToMiddle;;

			//			if (fabs(hitY - fusedData.robotState.mapCoordinate.y * JAFDSettings::Field::cellWidth) < JAFDSettings::MazeMapping::widthSecureDetectFactor * JAFDSettings::Field::cellWidth)
			//			{
			//				hitPointIsOk = true;
			//			}
			//		}

			//		if (hitPointIsOk)
			//		{
			//			Serial.println("LB");
			//		}
			//	}

			//	if (fusedData.distSensorState.rightFront == DistSensorStatus::ok)
			//	{
			//		bool hitPointIsOk = false;

			//		// Measurement is ok
			//		// Check if resulting hit point is a 90° wall on left side of us
			//		if (fusedData.robotState.heading == AbsoluteDir::north || fusedData.robotState.heading == AbsoluteDir::south)
			//		{
			//			float hitX = -sinf(fusedData.robotState.rotation.x) * fusedData.distances.rightFront / 10.0f + fusedData.robotState.position.x + cosf(fusedData.robotState.rotation.x + JAFDSettings::Mechanics::distSensLRAngleToMiddle) * JAFDSettings::Mechanics::distSensLRDistToMiddle;

			//			if (fabs(hitX - fusedData.robotState.mapCoordinate.x * JAFDSettings::Field::cellWidth) < JAFDSettings::MazeMapping::widthSecureDetectFactor * JAFDSettings::Field::cellWidth)
			//			{
			//				hitPointIsOk = true;
			//			}
			//		}
			//		else
			//		{
			//			float hitY = -cosf(fusedData.robotState.rotation.x) * fusedData.distances.rightFront / 10.0f + fusedData.robotState.position.y + sinf(fusedData.robotState.rotation.x + JAFDSettings::Mechanics::distSensLRAngleToMiddle) * JAFDSettings::Mechanics::distSensLRDistToMiddle;;

			//			if (fabs(hitY - fusedData.robotState.mapCoordinate.y * JAFDSettings::Field::cellWidth) < JAFDSettings::MazeMapping::widthSecureDetectFactor * JAFDSettings::Field::cellWidth)
			//			{
			//				hitPointIsOk = true;
			//			}
			//		}

			//		if (hitPointIsOk)
			//		{
			//			Serial.println("RF");
			//		}
			//	}

			//	if (fusedData.distSensorState.rightBack == DistSensorStatus::ok) 
			//	{
			//		bool hitPointIsOk = false;

			//		// Measurement is ok
			//		// Check if resulting hit point is a 90° wall on left side of us
			//		if (fusedData.robotState.heading == AbsoluteDir::north || fusedData.robotState.heading == AbsoluteDir::south)
			//		{
			//			float hitX = -sinf(fusedData.robotState.rotation.x) * fusedData.distances.rightBack / 10.0f + fusedData.robotState.position.x - cosf(fusedData.robotState.rotation.x + JAFDSettings::Mechanics::distSensLRAngleToMiddle) * JAFDSettings::Mechanics::distSensLRDistToMiddle;

			//			if (fabs(hitX - fusedData.robotState.mapCoordinate.x * JAFDSettings::Field::cellWidth) < JAFDSettings::MazeMapping::widthSecureDetectFactor * JAFDSettings::Field::cellWidth)
			//			{
			//				hitPointIsOk = true;
			//			}
			//		}
			//		else
			//		{
			//			float hitY = -cosf(fusedData.robotState.rotation.x) * fusedData.distances.rightBack / 10.0f + fusedData.robotState.position.y - sinf(fusedData.robotState.rotation.x + JAFDSettings::Mechanics::distSensLRAngleToMiddle) * JAFDSettings::Mechanics::distSensLRDistToMiddle;;

			//			if (fabs(hitY - fusedData.robotState.mapCoordinate.y * JAFDSettings::Field::cellWidth) < JAFDSettings::MazeMapping::widthSecureDetectFactor * JAFDSettings::Field::cellWidth)
			//			{
			//				hitPointIsOk = true;
			//			}
			//		}

			//		if (hitPointIsOk)
			//		{
			//			Serial.println("RB");
			//		}
			//	}
			//}

			lastDistances = currentDistances;
			lastDistSensStates = currentDistSensStates;
			lastPosition = fusedData.robotState.mapCoordinate;
			certainty = tempCertainty;
			cell = tempCell;
		}
	}
}
