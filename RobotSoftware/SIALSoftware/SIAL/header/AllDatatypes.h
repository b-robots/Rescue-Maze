#pragma once

#include <stdint.h>
#include "Vector.h"
#include "DuePinMapping.h"
#include "Math.h"

namespace SIAL
{
	enum class InterruptSource : uint8_t
	{
		pioA = ID_PIOA,
		pioB = ID_PIOB,
		pioC = ID_PIOC,
		pioD = ID_PIOD
	};

	enum class Victim : uint8_t
	{
		harmed,
		stable,
		unharmed,
		red,
		green,
		yellow,
		heat,
		none
	};

	enum class SerialType : uint8_t
	{
		software,
		zero,
		one,
		two,
		three
	};

	struct NewForcedFusionValues {
		bool clearCell = false;
		bool zeroPitch = false;
		bool newX = false;
		bool newY = false;
		bool newHeading = false;
		float x;
		float y;
		float heading;
	};

	struct FloatWheelSpeeds;

	// Speed of both wheels
	struct WheelSpeeds
	{
		int16_t left;
		int16_t right;

		explicit constexpr WheelSpeeds(int16_t left = 0, int16_t right = 0) : left(left), right(right) {}
		WheelSpeeds(const volatile WheelSpeeds& speeds) : left(speeds.left), right(speeds.right) {}
		constexpr WheelSpeeds(const WheelSpeeds& speeds) : left(speeds.left), right(speeds.right) {}
		explicit WheelSpeeds(const volatile FloatWheelSpeeds& speeds);
		explicit constexpr WheelSpeeds(const FloatWheelSpeeds& speeds);

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

		explicit constexpr FloatWheelSpeeds(float left = 0.0f, float right = 0.0f) : left(left), right(right) {}
		FloatWheelSpeeds(const volatile FloatWheelSpeeds& speeds) : left(speeds.left), right(speeds.right) {}
		constexpr FloatWheelSpeeds(const FloatWheelSpeeds& speeds) : left(speeds.left), right(speeds.right) {}
		explicit FloatWheelSpeeds(const volatile WheelSpeeds& speeds) : left(static_cast<float>(speeds.left)), right(static_cast<float>(speeds.right)) {}
		explicit constexpr FloatWheelSpeeds(const WheelSpeeds& speeds) : left(static_cast<float>(speeds.left)), right(static_cast<float>(speeds.right)) {}

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

	inline WheelSpeeds::WheelSpeeds(const volatile FloatWheelSpeeds& speeds) : left(static_cast<int16_t>(speeds.left)), right(static_cast<int16_t>(speeds.right)) {}
	inline constexpr WheelSpeeds::WheelSpeeds(const FloatWheelSpeeds& speeds) : left(static_cast<int16_t>(speeds.left)), right(static_cast<int16_t>(speeds.right)) {}

	// Which motor?
	enum class Motor : uint8_t
	{
		left,
		right
	};

	// Which heat sensor?
	enum class HeatSensorSide : uint8_t
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

	// Heading direction
	enum class AbsoluteDir : uint8_t
	{
		north = 1 << 0,
		east = 1 << 1,
		south = 1 << 2,
		west = 1 << 3
	};

	// Relative direction for robot
	enum class RelativeDir : uint8_t
	{
		forward,
		backward,
		left,
		right
	};

	// Coordinate on the map
	struct MapCoordinate
	{
		int8_t x;
		int8_t y;

		explicit constexpr MapCoordinate(int8_t x = 0, int8_t y = 0) : x(x), y(y) {}

		MapCoordinate getCoordinateInDir(AbsoluteDir dir) const {
			MapCoordinate result = *this;

			switch (dir)
			{
			case AbsoluteDir::north:
				result.x++;
				break;
			case AbsoluteDir::east:
				result.y--;
				break;
			case AbsoluteDir::south:
				result.x--;
				break;
			case AbsoluteDir::west:
				result.y++;
				break;
			default:
				break;
			}

			return result;
		}
	};

	// Home Position
	constexpr MapCoordinate homePosition = MapCoordinate{ 0, 0 };

	// Comparison operators for MapCoordinate
	inline bool operator==(const MapCoordinate& lhs, const MapCoordinate& rhs) { return (lhs.x == rhs.x && lhs.y == rhs.y); }
	inline bool operator!=(const MapCoordinate& lhs, const MapCoordinate& rhs) { return !(lhs == rhs); }

	inline RelativeDir makeRelative(const AbsoluteDir& absoluteDir, const AbsoluteDir heading)
	{
		switch (absoluteDir)
		{
		case AbsoluteDir::north:
		{
			switch (heading)
			{
			case AbsoluteDir::north:
				return RelativeDir::forward;
				break;
			case AbsoluteDir::east:
				return RelativeDir::left;
				break;
			case AbsoluteDir::south:
				return RelativeDir::backward;
				break;
			case AbsoluteDir::west:
				return RelativeDir::right;
				break;
			default:
				break;
			}

			break;
		}

		case AbsoluteDir::east:
		{
			switch (heading)
			{
			case AbsoluteDir::north:
				return RelativeDir::right;
				break;
			case AbsoluteDir::east:
				return RelativeDir::forward;
				break;
			case AbsoluteDir::south:
				return RelativeDir::left;
				break;
			case AbsoluteDir::west:
				return RelativeDir::backward;
				break;
			default:
				break;
			}

			break;
		}

		case AbsoluteDir::south:
		{
			switch (heading)
			{
			case AbsoluteDir::north:
				return RelativeDir::backward;
				break;
			case AbsoluteDir::east:
				return RelativeDir::right;
				break;
			case AbsoluteDir::south:
				return RelativeDir::forward;
				break;
			case AbsoluteDir::west:
				return RelativeDir::left;
				break;
			default:
				break;
			}

			break;
		}

		case AbsoluteDir::west:
		{
			switch (heading)
			{
			case AbsoluteDir::north:
				return RelativeDir::left;
				break;
			case AbsoluteDir::east:
				return RelativeDir::backward;
				break;
			case AbsoluteDir::south:
				return RelativeDir::right;
				break;
			case AbsoluteDir::west:
				return RelativeDir::forward;
				break;
			default:
				break;
			}

			break;
		}

		default:
			break;
		}
	}

	inline AbsoluteDir makeAbsolute(const RelativeDir& relativeDir, const AbsoluteDir heading)
	{
		switch (relativeDir)
		{
		case RelativeDir::forward:
		{
			switch (heading)
			{
			case AbsoluteDir::north:
				return AbsoluteDir::north;
				break;
			case AbsoluteDir::east:
				return AbsoluteDir::east;
				break;
			case AbsoluteDir::south:
				return AbsoluteDir::south;
				break;
			case AbsoluteDir::west:
				return AbsoluteDir::west;
				break;
			default:
				break;
			}

			break;
		}

		case RelativeDir::right:
		{
			switch (heading)
			{
			case AbsoluteDir::north:
				return AbsoluteDir::east;
				break;
			case AbsoluteDir::east:
				return AbsoluteDir::south;
				break;
			case AbsoluteDir::south:
				return AbsoluteDir::west;
				break;
			case AbsoluteDir::west:
				return AbsoluteDir::north;
				break;
			default:
				break;
			}

			break;
		}

		case RelativeDir::backward:
		{
			switch (heading)
			{
			case AbsoluteDir::north:
				return AbsoluteDir::south;
				break;
			case AbsoluteDir::east:
				return AbsoluteDir::west;
				break;
			case AbsoluteDir::south:
				return AbsoluteDir::north;
				break;
			case AbsoluteDir::west:
				return AbsoluteDir::east;
				break;
			default:
				break;
			}

			break;
		}

		case RelativeDir::left:
		{
			switch (heading)
			{
			case AbsoluteDir::north:
				return AbsoluteDir::west;
				break;
			case AbsoluteDir::east:
				return AbsoluteDir::north;
				break;
			case AbsoluteDir::south:
				return AbsoluteDir::east;
				break;
			case AbsoluteDir::west:
				return AbsoluteDir::south;
				break;
			default:
				break;
			}

			break;
		}

		default:
			break;
		}
	}

	// State of robot
	struct RobotState
	{
		FloatWheelSpeeds wheelSpeeds;	// Speed of the wheels
		float forwardVel;				// Forward velocity (cm/s)
		Vec2f position;					// Current position (cm)

		Vec3f angularVel;				// Angular velocity as (heading, pitch, bank) in Tait-Bryan angles with order (z, y', x') (rad/s)
		Vec3f forwardVec;				// Current Rotation as a forward vector
		float globalHeading;			// Heading relative to the beginning in rad including full turns [-inf; inf]
		float pitch;					// Pitch / Elevation in rad [-pi/2; pi/2]

		MapCoordinate mapCoordinate;	// Position on the map; (0, 0, 0) == start
		AbsoluteDir heading;			// Heading of the robot

		constexpr RobotState() : wheelSpeeds(), forwardVel(0.0f), position(), angularVel(), forwardVec(), heading(AbsoluteDir::north), globalHeading(0.0f), pitch(0.0f), mapCoordinate() {}
	};

	// Possible states of a cell
	namespace CellState
	{
		constexpr uint8_t visited = 1 << 0;
		constexpr uint8_t victim = 1 << 1;
		constexpr uint8_t checkpoint = 1 << 2;
		constexpr uint8_t blackTile = 1 << 3;
		constexpr uint8_t ramp = 1 << 4;
		constexpr uint8_t stair = 1 << 5;
		constexpr uint8_t bump = 1 << 6;
		constexpr uint8_t none = 0;
	}

	// Cell Connections
	namespace CellConnections
	{
		constexpr uint8_t directionMask = 0xf;
		constexpr uint8_t rampMask = 0xf0;
	}

	// Direction flags for normal entrance
	namespace EntranceDirections
	{
		constexpr uint8_t north = 1 << 0;
		constexpr uint8_t east = 1 << 1;
		constexpr uint8_t south = 1 << 2;
		constexpr uint8_t west = 1 << 3;
		constexpr uint8_t nowhere = 0;
	}

	// Direction flags for ramp entrance
	namespace RampDirections
	{
		constexpr uint8_t north = 1 << 4;
		constexpr uint8_t east = 1 << 5;
		constexpr uint8_t south = 1 << 6;
		constexpr uint8_t west = 1 << 7;
		constexpr uint8_t nowhere = 0;
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
		uint8_t cellConnections;

		// Information about Speed Bumps, Victims, Checkpoints...
		// From right to left:
		// 1.Bit: Is this Cell already visited?
		// 2.Bit: Victim already detected?
		// 3.Bit: Checkpoint?
		// 4.Bit: Black Tile?
		// 5.Bit: Ramp?
		// 6.Bit: Obstacle?
		// 7.Bit: Speed Bump?
		uint8_t cellState;

		explicit constexpr GridCell(uint8_t cellConnections = 0b0000, uint8_t cellState = 0) : cellConnections(cellConnections), cellState(cellState) {}
	};

	// Status of distance sensor
	enum class DistSensorStatus : uint8_t
	{
		ok,
		overflow,
		underflow,
		error
	};

	// All distance sensor states in one structure
	struct DistSensorStates
	{
		DistSensorStatus frontLeft;
		DistSensorStatus frontRight;
		DistSensorStatus frontLong;
		DistSensorStatus leftFront;
		DistSensorStatus leftBack;
		DistSensorStatus rightFront;
		DistSensorStatus rightBack;

		constexpr DistSensorStates() : frontLeft(DistSensorStatus::error), frontRight(DistSensorStatus::error), frontLong(DistSensorStatus::error), leftFront(DistSensorStatus::error), leftBack(DistSensorStatus::error), rightFront(DistSensorStatus::error), rightBack(DistSensorStatus::error) {}
	};

	struct DistSensBool
	{
		bool frontLeft;
		bool frontRight;
		bool frontLong;
		bool leftFront;
		bool leftBack;
		bool rightFront;
		bool rightBack;

		explicit constexpr DistSensBool(bool val = false) : frontLeft(val), frontRight(val), frontLong(val), leftFront(val), leftBack(val), rightFront(val), rightBack(val) {}
	};

	inline DistSensBool operator|(const DistSensBool& lhs, const DistSensBool& rhs) {
		DistSensBool result;
		result.frontLeft = lhs.frontLeft | rhs.frontLeft;
		result.frontRight = lhs.frontRight | rhs.frontRight;
		result.frontLong = lhs.frontLong | rhs.frontLong;
		result.leftFront = lhs.leftFront | rhs.leftFront;
		result.leftBack = lhs.leftBack | rhs.leftBack;
		result.rightFront = lhs.rightFront | rhs.rightFront;
		result.rightBack = lhs.rightBack | rhs.rightBack;
		return result;
	}

	inline bool operator==(const DistSensBool& lhs, const DistSensBool& rhs) {
		return (lhs.frontLeft == rhs.frontLeft &&
			lhs.frontRight == rhs.frontRight &&
			lhs.frontLong == rhs.frontLong &&
			lhs.leftFront == rhs.leftFront &&
			lhs.leftBack == rhs.leftBack &&
			lhs.rightFront == rhs.rightFront &&
			lhs.rightBack == rhs.rightBack);
	}

	// All distances in one structure
	struct Distances
	{
		uint16_t frontLeft;
		uint16_t frontRight;
		uint16_t frontLong;
		uint16_t leftFront;
		uint16_t leftBack;
		uint16_t rightFront;
		uint16_t rightBack;

		constexpr Distances() : frontLeft(0), frontRight(0), frontLong(0), leftFront(0), leftBack(0), rightFront(0), rightBack(0) {}
	};

	// Data from color sensor
	struct ColorSensData
	{
		uint16_t colorTemp;
		uint16_t lux;

		explicit constexpr ColorSensData(uint16_t colorTemp = 0, uint16_t lux = 0) : colorTemp(colorTemp), lux(lux) {}
	};

	enum class FloorTileColour : uint8_t {
		white,
		silver,
		black
	};

	struct FusedDistSensData {
		float distSensAngle;		// Angle measured by distance sensors (rad)
		float distSensAngleTrust;	// How much can I trust the measured angle? (0.0 - 1.0)
		DistSensBool fallingEdge;
		DistSensBool risingEdge;

		struct DistToWalls {
			float l = -1.0f;
			float r = -1.0f;
			float f = -1.0f;

			constexpr DistToWalls(const DistToWalls& data) : l(data.l), r(data.r), f(data.f){}
			explicit constexpr DistToWalls() : l(-1.0f), r(-1.0f), f(-1.0f){}

			inline const DistToWalls& operator=(const DistToWalls& data)
			{
				l = data.l;
				r = data.r;
				f = data.f;

				return *this;
			}
		} distToWalls;

		explicit constexpr FusedDistSensData() : distSensAngle(0.0f), distSensAngleTrust(0.0f), distToWalls(), fallingEdge(false), risingEdge(false) {}
	};

	// Data fused by SensorFusion
	struct FusedData
	{
		RobotState robotState;		// Current state of robot
		GridCell gridCell;			// Current grid cell
		Distances distances; 		// Results of distance measurement in mm
		DistSensorStates distSensorState;	// States of all distance sensors
		ColorSensData colorSensData;	// Data from color sensor at the bottom (includes color temperature and brightness in lux)
		DistSensBool distSensUpdates;
		FusedDistSensData fusedDistSens;	// Fused data of distance sensors (angle and distance to walls)

		constexpr FusedData() : robotState(), gridCell(), distances(), distSensorState(), colorSensData(), fusedDistSens(), distSensUpdates() {}
	};

	enum class DrivingTaskUID : uint8_t {
		invalid,
		stop,
		forceSpeed,
		rotate,
		followWall,
		alignWalls,
		followCell,
		ramp,
		stairs,
		rotationUnstuck
	};

	struct DrivingTaskInformation {
		bool finished = false;
		bool drivingStraight = false;
		DrivingTaskUID uid = DrivingTaskUID::invalid;
		RobotState startState;

		explicit DrivingTaskInformation(DrivingTaskUID _uid = DrivingTaskUID::invalid, bool _drivingStraight = false) : uid(_uid), finished(false), drivingStraight(_drivingStraight), startState() {}
	};
}