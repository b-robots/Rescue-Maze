/*
This file is responsible for all distance sensors
*/

#pragma once

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include <Adafruit_VL6180X.h>

#include "../../JAFDSettings.h"
#include "AllDatatypes.h"

namespace JAFD
{
	namespace DistanceSensors
	{
		enum class Status : uint8_t
		{
			noError,
			systemError,
			eceFailure,
			noConvergence,
			ignoringRange,
			noiseError,
			underflow,
			overflow
		};

		class DistanceSensor
		{
		public:
			virtual ReturnCode setup() = 0;
			virtual void updateValues() = 0;
			virtual float getDistance() const = 0;
			virtual Status getStatus() const = 0;
		};

		class VL6180 : public DistanceSensor
		{
		public:
			ReturnCode setup();
			void updateValues();
			float getDistance() const;
			Status getStatus() const;
		private:
			Adafruit_VL6180X _sensor;
			volatile float _distance;
			volatile Status _status;
		};

		class Lidar : public DistanceSensor
		{
		public:
			ReturnCode setup();
			void updateValues();
			float getDistance() const;
			Status getStatus() const;
		private:
			Adafruit_VL6180X _sensor;
			volatile float _distance;
			volatile Status _status;
		};

		extern VL6180 front;	// Front distance sensor
		extern Lidar left;		// Left distance sensor

		ReturnCode setup();
	}
}