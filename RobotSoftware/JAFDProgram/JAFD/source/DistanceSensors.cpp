/*
This file is responsible for all distance sensors
*/

#include "../../JAFDSettings.h"
#include "../header/DistanceSensors.h"
#include "../header/TCA9548A.h"

namespace JAFD
{
	namespace DistanceSensors
	{
		namespace
		{
			TCA9548A _i2cMultiplexer(JAFDSettings::DistanceSensors::multiplexerAddr);
		}

		// VL6180 class - begin
		VL6180::VL6180(uint8_t multiplexCh) : _multiplexCh(multiplexCh), _status(Status::noError) {}

		ReturnCode VL6180::setup() const
		{
			if (_i2cMultiplexer.getChannel() != _multiplexCh)
			{
				_i2cMultiplexer.selectChannel(_multiplexCh);
			}

			if (read8(_regModelID) != 0xB4) {
				// Retry
				delay(10);

				if (read8(_regModelID) != 0xB4) {
					return ReturnCode::error;
				}
			}

			loadSettings();

			write8(_regSysFreshOutOfReset, 0x00);

			return ReturnCode::ok;
		}

		void VL6180::loadSettings() const
		{
			if (_i2cMultiplexer.getChannel() != _multiplexCh)
			{
				_i2cMultiplexer.selectChannel(_multiplexCh);
			}

			// private settings from page 24 of app note
			write8(0x0207, 0x01);
			write8(0x0208, 0x01);
			write8(0x0096, 0x00);
			write8(0x0097, 0xfd);
			write8(0x00e3, 0x00);
			write8(0x00e4, 0x04);
			write8(0x00e5, 0x02);
			write8(0x00e6, 0x01);
			write8(0x00e7, 0x03);
			write8(0x00f5, 0x02);
			write8(0x00d9, 0x05);
			write8(0x00db, 0xce);
			write8(0x00dc, 0x03);
			write8(0x00dd, 0xf8);
			write8(0x009f, 0x00);
			write8(0x00a3, 0x3c);
			write8(0x00b7, 0x00);
			write8(0x00bb, 0x3c);
			write8(0x00b2, 0x09);
			write8(0x00ca, 0x09);
			write8(0x0198, 0x01);
			write8(0x01b0, 0x17);
			write8(0x01ad, 0x00);
			write8(0x00ff, 0x05);
			write8(0x0100, 0x05);
			write8(0x0199, 0x05);
			write8(0x01a6, 0x1b);
			write8(0x01ac, 0x3e);
			write8(0x01a7, 0x1f);
			write8(0x0030, 0x00);

			// Recommended : Public registers - See data sheet for more detail
			write8(0x0011, 0x10);       // Enables polling for 'New Sample ready'
										// when measurement completes
			write8(0x010a, 0x30);       // Set the averaging sample period
										// (compromise between lower noise and
										// increased execution time)
			write8(0x003f, 0x46);       // Sets the light and dark gain (upper
										// nibble). Dark gain should not be
										// changed.
			write8(0x0031, 0xFF);       // sets the # of range measurements after
										// which auto calibration of system is
										// performed
			write8(0x0040, 0x63);       // Set ALS integration time to 100ms
			write8(0x002e, 0x01);       // perform a single temperature calibration
										// of the ranging sensor

			// Optional: Public registers - See data sheet for more detail
			write8(0x001b, 0x09);       // Set default ranging inter-measurement
										// period to 100ms
			write8(0x003e, 0x31);       // Set default ALS inter-measurement period
										// to 500ms
			write8(0x0014, 0x24);       // Configures interrupt on 'New Sample
										// Ready threshold event'
		}

		// Write 1 byte
		void VL6180::write8(uint16_t address, uint8_t data) const
		{
			Wire.beginTransmission(_i2cAddr);
			Wire.write(address >> 8);
			Wire.write(address);
			Wire.write(data);
			Wire.endTransmission();
		}

		// Write 2 bytes
		void VL6180::write16(uint16_t address, uint16_t data) const
		{
			Wire.beginTransmission(_i2cAddr);
			Wire.write(address >> 8);
			Wire.write(address);
			Wire.write(data >> 8);
			Wire.write(data);
			Wire.endTransmission();
		}

		// Read 1 byte
		uint8_t VL6180::read8(uint16_t address) const
		{
			Wire.beginTransmission(_i2cAddr);
			Wire.write(address >> 8);
			Wire.write(address);
			Wire.endTransmission();

			Wire.requestFrom(_i2cAddr, (uint8_t)1);

			return Wire.read();
		}

		// Read 2 bytes
		uint16_t VL6180::read16(uint16_t address) const
		{
			uint16_t data;

			Wire.beginTransmission(_i2cAddr);
			Wire.write(address >> 8);
			Wire.write(address);
			Wire.endTransmission();

			Wire.requestFrom(_i2cAddr, (uint8_t)2);
			data = Wire.read();
			data <<= 8;
			data |= Wire.read();

			return data;
		}

		uint16_t VL6180::getDistance()
		{
			uint8_t distance;

			if (_i2cMultiplexer.getChannel() != _multiplexCh)
			{
				_i2cMultiplexer.selectChannel(_multiplexCh);
			}

			// Wait for device to be ready for range measurement
			while (!(read8(_regRangeStatus) & 0x01));

			// Start a range measurement
			write8(_regRangeStart, 0x01);

			// Poll until bit 2 is set
			while (!(read8(_regIntStatus) & 0x04));
			
			// Read range in mm
			distance = read8(_regRangeResult);

			// Clear interrupt
			write8(_regIntClear, 0x07);

			// Read status
			_status = static_cast<Status>(read8(_regRangeStatus) >> 4);
			
			if (_status == Status::noError || _status == Status::eceFailure || _status == Status::noiseError)
			{
				if (distance > maxDist) _status = Status::overflow;
				else if (distance < minDist) _status = Status::underflow;
			}
			else if (_status == Status::rawOverflow || _status == Status::overflow)
			{
				_status = Status::overflow;
			}
			else if (_status == Status::rawUnderflow || _status == Status::underflow)
			{
				_status = Status::underflow;
			}

			return distance;
		}

		VL6180::Status VL6180::getStatus() const
		{
			return _status;
		}

		// VL6180 class - end

		// TFMini class - begin

		TFMini::TFMini(SerialType serialType) : _serialType(serialType), _status(Status::noError), _distance(0) {}

		ReturnCode TFMini::setup()
		{
			switch (_serialType)
			{
			case JAFD::SerialType::zero:
				Serial.begin(_baudrate);
				_streamPtr = &Serial;
				break;

			case JAFD::SerialType::one:
				Serial1.begin(_baudrate);
				_streamPtr = &Serial1;
				break;

			case JAFD::SerialType::two:
				Serial2.begin(_baudrate);
				_streamPtr = &Serial2;
				break;

			case JAFD::SerialType::three:
				Serial3.begin(_baudrate);
				_streamPtr = &Serial3;
				break;

			default:
				return ReturnCode::error;
			}

			// Set to mm mode

			// Start config
			_streamPtr->write((uint8_t)0x42);
			_streamPtr->write((uint8_t)0x57);
			_streamPtr->write((uint8_t)0x02);
			_streamPtr->write((uint8_t)0x00);
			_streamPtr->write((uint8_t)0x00);
			_streamPtr->write((uint8_t)0x00);
			_streamPtr->write((uint8_t)0x01);
			_streamPtr->write((uint8_t)0x02);

			// mm Mode
			_streamPtr->write((uint8_t)0x42);
			_streamPtr->write((uint8_t)0x57);
			_streamPtr->write((uint8_t)0x02);
			_streamPtr->write((uint8_t)0x00);
			_streamPtr->write((uint8_t)0x00);
			_streamPtr->write((uint8_t)0x00);
			_streamPtr->write((uint8_t)0x00);
			_streamPtr->write((uint8_t)0x1A);

			// End config
			_streamPtr->write((uint8_t)0x42);
			_streamPtr->write((uint8_t)0x57);
			_streamPtr->write((uint8_t)0x02);
			_streamPtr->write((uint8_t)0x00);
			_streamPtr->write((uint8_t)0x00);
			_streamPtr->write((uint8_t)0x00);
			_streamPtr->write((uint8_t)0x00);
			_streamPtr->write((uint8_t)0x02);

			delay(100);

			return ReturnCode::ok;
		}
		
		TFMini::Status TFMini::takeMeasurement()
		{
			uint8_t numCharsRead = 0;
			uint8_t lastChar = 0x00;

			// Read the serial stream until we see the beginning of the TF Mini header, or we timeout reading too many characters.
			while (1)
			{

				if (_streamPtr->available())
				{
					uint8_t curChar = _streamPtr->read();

					if ((lastChar == 0x59) && (curChar == 0x59))
					{
						// Header start
						break;
					}
					else
					{
						// Continue reading
						lastChar = curChar;
						numCharsRead += 1;
					}
				}

				// Error detection:  If we read more than X characters without finding a frame header, then it's likely there is an issue with 
				// the Serial connection, and we should timeout and throw an error. 
				if (numCharsRead > _maxBytesBeforeHeader)
				{
					_status = Status::noSerialHeader;
					return _status;
				}

			}

			// Read one frame from the TFMini
			uint8_t frame[_frameSize];

			uint8_t checksum = 0x59 + 0x59;

			for (int i = 0; i < _frameSize; i++)
			{
				// Whait for character
				while (!_streamPtr->available());

				// Read character
				frame[i] = _streamPtr->read();

				// Store running checksum
				if (i < _frameSize - 2) {
					checksum += frame[i];
				}
			}

			// Compare checksum
			// Last byte in the frame is an 8-bit checksum 
			if (checksum != frame[_frameSize - 1])
			{
				_status = Status::badChecksum;
				return _status;
			}

			// Interpret frame
			uint16_t st = (frame[3] << 8) + frame[2];
			uint8_t reserved = frame[4];
			uint8_t originalSignalQuality = frame[5];
			_distance = (frame[1] << 8) + frame[0];

			// Return success
			_status = Status::noError;
			return _status;
		}

		uint16_t TFMini::getDistance()
		{
			uint8_t numMeasurementAttempts = 0;

			while (takeMeasurement() != Status::noError)
			{
				numMeasurementAttempts++;

				if (numMeasurementAttempts > _maxMeasurementTries) return 0;
			}

			if (_status == Status::noError)
			{
				if (_distance > maxDist) _status = Status::overflow;
				else if (_distance < minDist) _status = Status::underflow;
			}

			return _distance;
		}

		TFMini::Status TFMini::getStatus() const
		{
			return _status;
		}

		// TFMini class - end

		// VL53L0 class - begin

		VL53L0::VL53L0(uint8_t multiplexCh) : _multiplexCh(multiplexCh), _status(Status::undefinedError) {}

		ReturnCode VL53L0::setup()
		{
			if (_i2cMultiplexer.getChannel() != _multiplexCh)
			{
				_i2cMultiplexer.selectChannel(_multiplexCh);
			}

			_sensor.setTimeout(500);
			if (_sensor.init()) return ReturnCode::ok;
			else return ReturnCode::error;
		}

		uint16_t VL53L0::getDistance()
		{
			if (_i2cMultiplexer.getChannel() != _multiplexCh)
			{
				_i2cMultiplexer.selectChannel(_multiplexCh);
			}

			uint16_t distance = _sensor.readRangeSingleMillimeters();

			_status = Status::noError;
			
			if (_sensor.timeoutOccurred())
			{
				_status = Status::timeOut;
			}
			else
			{
				if (distance > maxDist) _status = Status::overflow;
				else if (distance < minDist) _status = Status::underflow;
			}

			return distance;
		}

		VL53L0::Status VL53L0::getStatus() const
		{
			return _status;
		}

		// VL53L0 class - end

		VL53L0 frontLeft(JAFDSettings::DistanceSensors::FrontLeft::multiplexCh);
		VL53L0 frontRight(JAFDSettings::DistanceSensors::FrontRight::multiplexCh);
		TFMini frontLong(JAFDSettings::DistanceSensors::FrontLong::serialType);
		TFMini backLong(JAFDSettings::DistanceSensors::BackLong::serialType);
		VL6180 leftFront(JAFDSettings::DistanceSensors::LeftFront::multiplexCh);
		VL6180 leftBack(JAFDSettings::DistanceSensors::LeftBack::multiplexCh);
		VL6180 rightFront(JAFDSettings::DistanceSensors::RightFront::multiplexCh);
		VL6180 rightBack(JAFDSettings::DistanceSensors::LeftFront::multiplexCh);

		ReturnCode setup()
		{
			ReturnCode code = ReturnCode::ok;

			if (leftFront.setup() != ReturnCode::ok)
			{
				code = ReturnCode::fatalError;
			}

			if (leftBack.setup() != ReturnCode::ok)
			{
				code = ReturnCode::fatalError;
			}

			if (rightFront.setup() != ReturnCode::ok)
			{
				code = ReturnCode::fatalError;
			}

			if (rightBack.setup() != ReturnCode::ok)
			{
				code = ReturnCode::fatalError;
			}

			if (frontLeft.setup() != ReturnCode::ok)
			{
				code = ReturnCode::fatalError;
			}

			if (frontRight.setup() != ReturnCode::ok)
			{
				code = ReturnCode::fatalError;
			}

			//if (frontLong.setup() != ReturnCode::ok)
			//{
			//	code = ReturnCode::fatalError;
			//}

			//if (backLong.setup() != ReturnCode::ok)
			//{
			//	code = ReturnCode::fatalError;
			//}

			return code;
		}
	}
}