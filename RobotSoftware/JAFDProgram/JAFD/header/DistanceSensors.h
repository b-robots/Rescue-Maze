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
#include <DFRobot_TFmini.h>

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
			virtual uint16_t getDistance() const = 0;
		protected:
			volatile uint16_t _distance;
		};

		class VL6180 : public DistanceSensor
		{
		public:
			ReturnCode setup();
			void updateValues();
			uint16_t getDistance() const;
			Status getStatus() const;
		private:
			Adafruit_VL6180X _sensor;
			volatile Status _status;
		};

		class TFMini : public DistanceSensor
		{
		public:
			ReturnCode setup();
			void updateValues();
			uint16_t getDistance() const;
		private:
			DFRobot_TFmini _sensor;
		};

		extern VL6180 frontLeft;	// Front-Left short distance sensor
		extern VL6180 frontRight;	// Front-Right short distance sensor
		extern TFMini frontLong;		// Front long distance sensor
		extern TFMini backLong;		// Back long distance sensor
		extern VL6180 leftFront;	// Left-Front short distance sensor
		extern VL6180 leftBack;		// Left-Back short distance sensor
		extern VL6180 rightFront;	// Right-Front short distance sensor
		extern VL6180 rightBack;	// Right-Front short distance sensor

		ReturnCode setup();
	}
}