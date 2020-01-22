/*
In this file are all definitions and includes for datatypes and name aliases, which are not specialized for one module
*/

#pragma once

#include <stdint.h>
#include "Vector.h"

namespace JAFD
{
	enum class SerialType : uint8_t
	{
		software,
		zero,
		one,
		two,
		three
	};

	struct FloatWheelSpeeds;

	// Speed of both wheels
	struct WheelSpeeds
	{
		int16_t left;
		int16_t right;

		WheelSpeeds(int16_t left = 0, int16_t right = 0) : left(left), right(right) {}
		WheelSpeeds(const volatile WheelSpeeds& speeds) : left(speeds.left), right(speeds.right) {}
		WheelSpeeds(const WheelSpeeds& speeds) : left(speeds.left), right(speeds.right) {}
		explicit WheelSpeeds(const volatile FloatWheelSpeeds speeds);

		inline const volatile WheelSpeeds& operator=(const volatile WheelSpeeds speeds) volatile
		{
			left = speeds.left;
			right = speeds.right;

			return *this;
		}

		inline const WheelSpeeds& operator=(const WheelSpeeds& speeds)
		{
			left = speeds.left;
			right = speeds.right;

			return *this;
		}
	};

	// More accurate speed of both wheels
	struct FloatWheelSpeeds
	{
		float left;
		float right;

		FloatWheelSpeeds(float left = 0.0f, float right = 0.0f) : left(left), right(right) {}
		FloatWheelSpeeds(const volatile FloatWheelSpeeds& speeds) : left(speeds.left), right(speeds.right) {}
		FloatWheelSpeeds(const FloatWheelSpeeds& speeds) : left(speeds.left), right(speeds.right) {}
		explicit FloatWheelSpeeds(const volatile WheelSpeeds speeds) : left(static_cast<float>(speeds.left)), right(static_cast<float>(speeds.right)) {}

		inline const volatile FloatWheelSpeeds& operator=(const volatile FloatWheelSpeeds speeds) volatile
		{
			left = speeds.left;
			right = speeds.right;

			return *this;
		}

		inline const FloatWheelSpeeds& operator=(const FloatWheelSpeeds& speeds)
		{
			left = speeds.left;
			right = speeds.right;

			return *this;
		}
	};

	inline WheelSpeeds::WheelSpeeds(const volatile FloatWheelSpeeds speeds) : left(static_cast<int16_t>(speeds.left)), right(static_cast<int16_t>(speeds.right)) {}

	// Which motor?
	enum class Motor : uint8_t
	{
		left,
		right
	};

	// Return codes of a function
	enum class ReturnCode : uint8_t
	{
		fatalError,
		error,
		aborted,
		ok
	};

	// Coordinate on the map
	struct MapCoordinate
	{
		int8_t x;
		int8_t y;
		uint8_t floor;

		MapCoordinate(int8_t x = 0, int8_t y = 0, uint8_t floor = 0) : x(x), y(y), floor(floor) {}
		MapCoordinate(const volatile MapCoordinate& coor) : x(coor.x), y(coor.y), floor(coor.floor) {}
		MapCoordinate(const MapCoordinate& coor) : x(coor.x), y(coor.y), floor(coor.floor) {}
	};

	// Home Position
	constexpr MapCoordinate homePosition = { 0, 0, 0 };

	// Comparison operators for MapCoordinate
	inline bool operator==(const MapCoordinate& lhs, const MapCoordinate& rhs) { return (lhs.floor == rhs.floor && lhs.x == rhs.x && lhs.y == rhs.y); }
	inline bool operator!=(const MapCoordinate& lhs, const MapCoordinate& rhs) { return !(lhs == rhs); }
	// State of robot
	struct RobotState
	{
		FloatWheelSpeeds wheelSpeeds;	// Speed of the wheels
		float forwardVel;				// Forward velocity (cm/s)
		Vec3f position;					// Current position (cm)

		Vec3f angularVel;				// Angular velocity as { yaw (= steering angle) / pitch (= tilt) / roll (= lean angle) } (rad/s)
		Vec3f rotation;					// Current Rotation as { yaw (= steering angle) / pitch (= tilt) / roll (= lean angle) } (rad)
		
		MapCoordinate mapCoordinate;	// Position on the map; (0, 0, 0) == start

		RobotState(FloatWheelSpeeds wheelSpeeds = FloatWheelSpeeds(), float forwardVel = 0.0f, Vec3f position = Vec3f(), Vec3f angularVel = Vec3f(), Vec3f rotation = Vec3f()) : wheelSpeeds(wheelSpeeds), forwardVel(forwardVel), position(position), angularVel(angularVel), rotation(rotation) {}
		RobotState(const volatile RobotState& state) : wheelSpeeds(state.wheelSpeeds), forwardVel(state.forwardVel), position(state.position), angularVel(state.angularVel), rotation(state.rotation) {}
		RobotState(const RobotState& state) : wheelSpeeds(state.wheelSpeeds), forwardVel(state.forwardVel), position(state.position), angularVel(state.angularVel), rotation(state.rotation) {}

		inline const volatile RobotState& operator=(const volatile RobotState state) volatile
		{
			wheelSpeeds = state.wheelSpeeds;
			forwardVel = state.forwardVel;
			position = state.position;
			angularVel = state.angularVel;
			rotation = state.rotation;

			return *this;
		}

		inline const RobotState& operator=(const RobotState& state)
		{
			wheelSpeeds = state.wheelSpeeds;
			forwardVel = state.forwardVel;
			position = state.position;
			angularVel = state.angularVel;
			rotation = state.rotation;

			return *this;
		}
	};

	// Possible states of a cell
	namespace CellState
	{
		constexpr uint8_t visited = 1 << 0;
		constexpr uint8_t victim = 1 << 1;
		constexpr uint8_t checkpoint = 1 << 2;
		constexpr uint8_t blackTile = 1 << 3;
		constexpr uint8_t ramp = 1 << 4;
		constexpr uint8_t none = 0;
	}

	// Cell Connections
	namespace CellConnections
	{
		constexpr uint8_t directionMask = 0xf;
		constexpr uint8_t rampMask = 0x10;
	}

	// Direction flags
	namespace Direction
	{
		constexpr uint8_t north = 1 << 0;
		constexpr uint8_t east = 1 << 1;
		constexpr uint8_t south = 1 << 2;
		constexpr uint8_t west = 1 << 3;
		constexpr uint8_t nowhere = 0;
	}

	// Ramp Direction
	namespace RampDirection
	{
		constexpr uint8_t north = 0b00 << 4;
		constexpr uint8_t east = 0b01 << 4;
		constexpr uint8_t south = 0b11 << 4;
		constexpr uint8_t west = 0b11 << 4;
	}

	// Informations for one cell
	struct GridCell
	{
		// Information about the entrances of the cell
		// From right to left:
		// 1.Bit: Entrance North
		// 2.Bit: Entrance East
		// 3.Bit: Entrance South
		// 4.Bit: Entrance West
		// 5. & 6.Bit: 00 = Ramp North; 01 = Ramp East; 10 = Ramp South; 11 = Ramp West
		// 7. & 8.Bit: Unused
		uint8_t cellConnections;

		// Information about Speed Bumps, Victims, Checkpoints...
		// From right to left:
		// 1.Bit: Is this Cell already visited?
		// 2.Bit: Victim already detected?
		// 3.Bit: Checkpoint?
		// 4.Bit: Black Tile?
		// 5.Bit: Ramp?
		uint8_t cellState;

		GridCell(uint8_t cellConnections = 0, uint8_t cellState = 0) : cellConnections(cellConnections), cellState(cellState) {}
		GridCell(const volatile GridCell& cell) : cellConnections(cell.cellConnections), cellState(cell.cellState) {}
		GridCell(const GridCell& cell) : cellConnections(cell.cellConnections), cellState(cell.cellState) {}
	};

	// Data fused by SensorFusion
	struct FusedData
	{
		RobotState robotState;	// Current state of robot
		GridCell gridCell;		// Current grid cell
	};
}