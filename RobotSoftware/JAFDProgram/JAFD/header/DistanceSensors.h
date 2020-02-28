/*
This file is responsible for all distance sensors
*/

#pragma once

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include <Wire.h>
#include <Adafruit_VL53L0X.h>

#include "../../JAFDSettings.h"
#include "AllDatatypes.h"

namespace JAFD
{
	namespace DistanceSensors
	{
		class VL6180
		{
		public:
			enum class Status : uint8_t
			{
				noError = 0,		// Success
				systemError1 = 1,	// System error
				systemError5 = 5,	// System error
				eceFailure = 6,		// Early convergence estimate fail
				noConvergence = 7,	// No target detected
				ignoringRange = 8,	// Ignore threshold check faild
				noiseError = 11,	// Too noise ambient
				rawUnderflow = 12,	// Raw measurement underflow
				rawOverflow = 13,	// Raw measurement overflow
				underflow = 14,		// Measurement underflow
				overflow = 15,		// Measurement overflow
				outOfRange			// Measurement is out of range
			};

			static const uint16_t minDist = 5;
			static const uint16_t maxDist = 150;

			VL6180(uint8_t multiplexCh);
			ReturnCode setup() const;
			uint8_t getDistance();		// Get distance in mm
			Status getStatus() const;

		private:
			// Register addresses
			static const uint8_t _regModelID = 0x000;				// Device model identification
			static const uint8_t _regIntConfig = 0x014;				// Interrupt configuration
			static const uint8_t _regIntClear = 0x015;				// Interrupt clear
			static const uint8_t _regSysFreshOutOfReset = 0x016;	// Fresh out of reset
			static const uint8_t _regRangeStart = 0x018;			// Start rang measurement
			static const uint8_t _regLuxStart = 0x038;				// Start lux reading
			static const uint8_t _regLuxGain = 0x03F;				// Lux gain
			static const uint8_t _regIntegrationPerHi = 0x040;		// Integration period for ALS mode - high byte
			static const uint8_t _regIntegrationPerLo = 0x041;		// Integration period for ALS mode - low byte
			static const uint8_t _regRangeStatus = 0x04d;			// Error codes
			static const uint8_t _regIntStatus = 0x04f;				// Interrupt status
			static const uint8_t _regALSResult = 0x050;				// Light reading result
			static const uint8_t _regRangeResult = 0x062;			// Range reading result

			// ALS - Gains
			static const uint8_t _alsGain1 = 0x06;		// x 1
			static const uint8_t _alsGain1_25 = 0x05;	// x 1.25
			static const uint8_t _alsGain1_67 = 0x04;	// x 1.67
			static const uint8_t _alsGain2_5 = 0x03;	// x 2.5
			static const uint8_t _alsGain5 = 0x02;		// x 5
			static const uint8_t _alsGain10 = 0x01;		// x 10
			static const uint8_t _alsGain20 = 0x00;		// x 20
			static const uint8_t _alsGain40 = 0x07;		// x 40

			static const uint8_t _i2cAddr = 0x29;

			const uint8_t _multiplexCh;
			Status _status;

			void loadSettings() const;
			void write8(uint16_t address, uint8_t data) const;
			void write16(uint16_t address, uint16_t data) const;
			uint16_t read16(uint16_t address) const;
			uint8_t read8(uint16_t address) const;
		};

		class TFMini
		{
		public:
			enum class Status : uint8_t
			{
				noError,		// Success
				noSerialHeader,	// No serial header read
				badChecksum,	// Checksum doesn`t match
				outOfRange		// Range error
			};

			static const uint16_t minDist = 300;
			static const uint16_t maxDist = 12000;

			ReturnCode setup();
			uint16_t getDistance();	// Get distance in mm
			Status getStatus() const;
			TFMini(SerialType serialType);

		private:
			static const uint32_t _baudrate = 115200;
			static const uint8_t _maxBytesBeforeHeader = 30;
			static const uint8_t _frameSize = 7;
			static const uint8_t _maxMeasurementTries = 5;

			const SerialType _serialType;
			Stream* _streamPtr;
			uint16_t _distance;
			Status _status;

			Status takeMeasurement();
		};

		class VL53L0
		{
		public:
			enum class Status : uint8_t
			{
				noError,				// Success
				calibrationError,		// Problem with calibration data
				underflow,				// Value too low -> clipped to min
				undefinedError,			// Undefined error
				functionUnavailable,	// Requested functionality is (currently) not available
				rangeError,				// Ranging error
				timeOut,				// time out during measurement
				bufferTooSmall,			// buffer is to small
				ioError,				// Error with GPIO-Functionality
				interruptError,			// Error during interrupt clear
				divisionByZero,			// Division by zero
				spadInitError,			// Error during SPAD initialization
				outOfRange				// Out of range
			};

			static const uint16_t minDist = 50;
			static const uint16_t maxDist = 1200;

			ReturnCode setup();
			uint16_t getDistance();		// Get distance in mm
			Status getStatus() const;
			VL53L0(uint8_t multiplexCh);

		private:
			const uint8_t _multiplexCh;

			Adafruit_VL53L0X _sensor;
			Status _status;
		};

		extern VL53L0 frontLeft;	// Front-Left short distance sensor
		extern VL53L0 frontRight;	// Front-Right short distance sensor
		extern TFMini frontLong;	// Front long distance sensor
		extern TFMini backLong;		// Back long distance sensor
		extern VL6180 leftFront;	// Left-Front short distance sensor
		extern VL6180 leftBack;		// Left-Back short distance sensor
		extern VL6180 rightFront;	// Right-Front short distance sensor
		extern VL6180 rightBack;	// Right-Front short distance sensor
		extern VL6180 packsRight;
		extern VL6180 packsLeft;

		ReturnCode setup();
	}
}