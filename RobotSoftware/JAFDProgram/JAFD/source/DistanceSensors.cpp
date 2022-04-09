/*
This file is responsible for all distance sensors
*/

#include "../../JAFDSettings.h"
#include "../header/DistanceSensors.h"
#include "../header/TCA9548A.h"
#include "../header/SpiNVSRAM.h"
#include "../header/SensorFusion.h"
#include "../header/SmallThings.h"

namespace JAFD
{
	namespace DistanceSensors
	{
		// VL6180 class - begin
		VL6180::VL6180(uint8_t multiplexCh, uint8_t id) : _multiplexCh(multiplexCh), _status(Status::unknownError), _id(id) {}

		ReturnCode VL6180::setup() const
		{
			if (I2CMultiplexer::getChannel() != _multiplexCh)
			{
				I2CMultiplexer::selectChannel(_multiplexCh);
			}

			if (read8(_regModelID) != 0xB4) {
				// Retry
				if (read8(_regModelID) != 0xB4) {
					return ReturnCode::error;
				}
			}

			loadSettings();

			write8(_regSysFreshOutOfReset, 0x00);

			return ReturnCode::ok;
		}

		void VL6180::calcCalibData(uint16_t firstTrue, uint16_t firstMeasure, uint16_t secondTrue, uint16_t secondMeasure)
		{
			if (abs((int32_t)secondMeasure - firstMeasure) < JAFDSettings::DistanceSensors::minCalibDataDiff)
			{
				_k = 1.0f;
			}
			else
			{
				_k = (float)((int32_t)secondTrue - firstTrue) / (float)((int32_t)secondMeasure - firstMeasure);

				if (_k < 0.0f) _k = 1.0f;
			}

			_d = (int16_t)(((firstTrue - _k * firstMeasure) + (secondTrue - _k * secondMeasure)) / 2.0f);

			Serial.println(_k);
			Serial.println(_d);
		}

		void VL6180::storeCalibData()
		{
			uint32_t startAddr = JAFDSettings::SpiNVSRAM::distSensStartAddr + JAFDSettings::DistanceSensors::bytesPerCalibData * _id;

			uint16_t storeK = _k * 100.0f;
			uint16_t storeD = ((uint16_t)(abs(_d)) & 0x7fff) | ((_d < 0) ? 1 << 15 : 0);

			SpiNVSRAM::writeByte(startAddr, storeK & 0xff);
			SpiNVSRAM::writeByte(startAddr + 1, storeK >> 8);
			SpiNVSRAM::writeByte(startAddr + 2, storeD & 0xff);
			SpiNVSRAM::writeByte(startAddr + 3, storeD >> 8);
		}

		void VL6180::restoreCalibData()
		{
			uint32_t startAddr = JAFDSettings::SpiNVSRAM::distSensStartAddr + JAFDSettings::DistanceSensors::bytesPerCalibData * _id;

			uint16_t storeK = SpiNVSRAM::readByte(startAddr) | (SpiNVSRAM::readByte(startAddr + 1) << 8);
			_k = storeK / 100.0f;

			uint16_t storeD = SpiNVSRAM::readByte(startAddr + 2) | ((uint16_t)SpiNVSRAM::readByte(startAddr + 3) << 8);
			_d = (storeD & 0x7fff) * ((storeD >> 15) ? -1 : 1);

			if (_k > 1.5f || _k < 0.5f || _d > 100) {
				Serial.println("No useful distance sensor calibration data could be restored!");
				resetCalibData();
				storeCalibData();
			}

			resetCalibData();
			storeCalibData();
		}

		void VL6180::resetCalibData()
		{
			_k = 1.0f;
			_d = 0;
		}

		void VL6180::loadSettings() const
		{
			if (I2CMultiplexer::getChannel() != _multiplexCh)
			{
				I2CMultiplexer::selectChannel(_multiplexCh);
			}

			// Private settings from page 24 of app note
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
			write8(0x0011, 0x10);	// Enables polling for ‘New Sample ready’
									// when measurement completes
			write8(0x010a, 48);		// Set the averaging sample period
									// (compromise between lower noise and
									// increased execution time)
			write8(0x003f, 0x46);	// Sets the light and dark gain (upper
									// nibble). Dark gain should not be
									// changed.
			write8(0x0031, 0xFF);	// sets the # of range measurements after
									// which auto calibration of system is
									// performed
			write8(0x002e, 0x01);	// perform a single temperature calibration
									// of the ranging sensor
			write8(0x0014, 0x24);	// Configures interrupt on ‘New Sample
									// Ready threshold event’

			delay(10);

			// 25 Hz continuos mode
			write8(0x001c, 30);		// max convergence time = 30ms
			write8(0x001b, 3);		// inter measurement period = 40ms (= 3 * 10ms + 10ms)

			// Select single mode (to stop eventual continuous)
			write8(_regRangeStart, 0x00);
			write8(_regRangeStart, 0x01);

			delay(100);

			// Start continuous mode
			write8(_regRangeStart, 0x03);
		}

		void VL6180::setKD(float k, int d)
		{
			_k = k;
			_d = d;
		}

		// Write 1 byte
		void VL6180::write8(uint16_t address, uint8_t data) const
		{
			Wire.beginTransmission(_i2cAddr);
			Wire.write(address >> 8);
			Wire.write(address);
			Wire.write(data);
			Wire.endTransmission(true);
		}

		// Write 2 bytes
		void VL6180::write16(uint16_t address, uint16_t data) const
		{
			Wire.beginTransmission(_i2cAddr);
			Wire.write(address >> 8);
			Wire.write(address);
			Wire.write(data >> 8);
			Wire.write(data);
			Wire.endTransmission(true);
		}

		// Read 1 byte
		uint8_t VL6180::read8(uint16_t address) const
		{
			Wire.beginTransmission(_i2cAddr);
			Wire.write(address >> 8);
			Wire.write(address);
			Wire.endTransmission(true);

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
			Wire.endTransmission(true);

			Wire.requestFrom(_i2cAddr, (uint8_t)2);
			data = Wire.read();
			data <<= 8;
			data |= Wire.read();

			return data;
		}

		uint16_t VL6180::getDistance()
		{
			uint16_t distance;

			if (I2CMultiplexer::getChannel() != _multiplexCh)
			{
				I2CMultiplexer::selectChannel(_multiplexCh);
			}

			// Poll until data is available
			auto startMillis = millis();
			uint8_t state;

			do
			{
				state = read8(_regIntStatus);

				if (millis() - startMillis > JAFDSettings::DistanceSensors::timeout)
				{
					clearInterrupt();

					// TODO: try to recover

					// Select single mode (to stop eventual continuous)
					write8(_regRangeStart, 0x00);
					write8(_regRangeStart, 0x01);

					delay(100);

					// Start continuous mode
					write8(_regRangeStart, 0x03);

					delay(50);

					if (read8(_regModelID) != 0xB4)
					{
						Serial.println("I2C problem");
						I2CBus::resetBus();
					}

					Serial.print("to");
					Serial.println(_id);

					// Timeout
					_status = static_cast<Status>(read8(_regRangeStatus) >> 4);
					_status = Status::timeout;
					return 0;
				}
			} while (state == 255 || (state & 0b111) != 4 || (state & 0b11000000 == 0 && (state & 0b111) != 4));

			// Read range in mm
			float tempDist = read8(_regRangeResult) * _k + _d;

			if (tempDist < 0.0f) distance = 0;
			else distance = (uint16_t)roundf(tempDist);

			// Clear interrupt
			clearInterrupt();

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

		void VL6180::clearInterrupt()
		{
			if (I2CMultiplexer::getChannel() != _multiplexCh)
			{
				I2CMultiplexer::selectChannel(_multiplexCh);
			}

			// Clear interrupt
			write8(_regIntClear, 0x05);
		}

		VL6180::Status VL6180::getStatus() const
		{
			return _status;
		}

		// VL6180 class - end

		// TFMini class - begin

		TFMini::TFMini(SerialType serialType, uint8_t id) : _serialType(serialType), _status(Status::noError), _distance(0), _id(id) {}

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

			delay(100);

			uint8_t numCharsRead = 0;
			uint8_t lastChar = 0x00;
			auto startMillis = millis();

			while (true)
			{
				if (millis() - startMillis > JAFDSettings::DistanceSensors::timeout)
				{
					return ReturnCode::error;
				}

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

				if (numCharsRead > _maxBytesBeforeHeader)
				{
					return ReturnCode::error;
				}
			}

			return ReturnCode::ok;
		}

		TFMini::Status TFMini::takeMeasurement()
		{
			uint8_t numCharsRead = 0;
			uint8_t lastChar = 0x00;

			// Read the serial stream until we see the beginning of the TF Mini header, or we timeout reading too many characters.
			auto startMillis = millis();
			while (1)
			{
				if (millis() - startMillis > JAFDSettings::DistanceSensors::timeout / _maxMeasurementTries)
				{
					_distance = 0;
					_status = Status::noSerialHeader;
					return _status;
				}

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

			_distance = ((frame[1] << 8) + frame[0]) * 10;		// cm to mm

			// Return success
			_status = Status::noError;

			return _status;
		}

		uint16_t TFMini::getDistance()
		{
			uint8_t numMeasurementAttempts = 0;

			do
			{
				numMeasurementAttempts++;

				if (numMeasurementAttempts > _maxMeasurementTries)
				{
					clearSerialBuffer();
					return 0;
				}

				takeMeasurement();
			} while (_status != Status::noError);

			clearSerialBuffer();

			float tempDist = _distance * _k + _d;

			if (tempDist < 0.0f) _distance = 0;
			else _distance = (uint16_t)roundf(tempDist);

			if (_distance >= maxDist) _status = Status::overflow;
			else if (_distance <= minDist) _status = Status::underflow;

			return _distance;
		}

		void TFMini::calcCalibData(uint16_t firstTrue, uint16_t firstMeasure, uint16_t secondTrue, uint16_t secondMeasure)
		{
			if (abs((int32_t)secondMeasure - firstMeasure) < JAFDSettings::DistanceSensors::minCalibDataDiff)
			{
				_k = 1.0f;
			}
			else
			{
				_k = (float)((int32_t)secondTrue - firstTrue) / (float)((int32_t)secondMeasure - firstMeasure);

				if (_k < 0.0f) _k = 1.0f;
			}

			_d = (int16_t)(((firstTrue - _k * firstMeasure) + (secondTrue - _k * secondMeasure)) / 2.0f);
		}

		void TFMini::storeCalibData()
		{
			uint32_t startAddr = JAFDSettings::SpiNVSRAM::distSensStartAddr + JAFDSettings::DistanceSensors::bytesPerCalibData * _id;

			uint16_t storeK = _k * 100.0f;
			uint16_t storeD = ((uint16_t)(abs(_d)) & 0x7fff) | ((_d < 0) ? 1 << 15 : 0);

			SpiNVSRAM::writeByte(startAddr, storeK & 0xff);
			SpiNVSRAM::writeByte(startAddr + 1, storeK >> 8);
			SpiNVSRAM::writeByte(startAddr + 2, storeD & 0xff);
			SpiNVSRAM::writeByte(startAddr + 3, storeD >> 8);
		}

		void TFMini::restoreCalibData()
		{
			uint32_t startAddr = JAFDSettings::SpiNVSRAM::distSensStartAddr + JAFDSettings::DistanceSensors::bytesPerCalibData * _id;

			uint16_t storeK = SpiNVSRAM::readByte(startAddr) | (SpiNVSRAM::readByte(startAddr + 1) << 8);
			_k = storeK / 100.0f;

			uint16_t storeD = SpiNVSRAM::readByte(startAddr + 2) | ((uint16_t)SpiNVSRAM::readByte(startAddr + 3) << 8);
			_d = (storeD & 0x7fff) * ((storeD >> 15) ? -1 : 1);

			if (_k > 1.5f || _k < 0.5f || _d > 100) {
				Serial.println("No useful distance sensor calibration data could be restored!");
				resetCalibData();
				storeCalibData();
			}
		}

		void TFMini::resetCalibData()
		{
			_k = 1.0f;
			_d = 0;
		}

		TFMini::Status TFMini::getStatus() const
		{
			return _status;
		}

		void TFMini::clearSerialBuffer()
		{
			while (_streamPtr->available()) volatile auto temp = _streamPtr->read();
		}

		// TFMini class - end

		// VL53L0 class - begin

		VL53L0::VL53L0(uint8_t multiplexCh, uint8_t id) : _multiplexCh(multiplexCh), _status(Status::undefinedError), _id(id) {}

		ReturnCode VL53L0::setup()
		{
			if (I2CMultiplexer::getChannel() != _multiplexCh)
			{
				I2CMultiplexer::selectChannel(_multiplexCh);
			}
			_sensor.setTimeout(JAFDSettings::DistanceSensors::timeout);

			if (!_sensor.init()) return ReturnCode::error;

			_sensor.startContinuous();

			return ReturnCode::ok;
		}

		uint16_t VL53L0::getDistance()
		{
			if (I2CMultiplexer::getChannel() != _multiplexCh)
			{
				I2CMultiplexer::selectChannel(_multiplexCh);
			}

			uint16_t distance = _sensor.readRangeContinuousMillimeters();

			float tempDist = distance * _k + _d;

			if (tempDist < 0.0f) distance = 0;
			else distance = (uint16_t)roundf(tempDist);

			_status = Status::noError;

			if (_sensor.timeoutOccurred())
			{
				Serial.print("to");
				Serial.println(_id);
				_status = Status::timeOut;
				I2CBus::resetBus();
			}
			else
			{
				if (distance > maxDist) _status = Status::overflow;
				else if (distance < minDist) _status = Status::underflow;
			}

			return distance;
		}

		void VL53L0::clearInterrupt()
		{
			if (I2CMultiplexer::getChannel() != _multiplexCh)
			{
				I2CMultiplexer::selectChannel(_multiplexCh);
			}

			_sensor.writeReg(_sensor.SYSTEM_INTERRUPT_CLEAR, 0x01);
		}

		void VL53L0::calcCalibData(uint16_t firstTrue, uint16_t firstMeasure, uint16_t secondTrue, uint16_t secondMeasure)
		{
			if (abs((int32_t)secondMeasure - firstMeasure) < JAFDSettings::DistanceSensors::minCalibDataDiff)
			{
				_k = 1.0f;
			}
			else
			{
				_k = (float)((int32_t)secondTrue - firstTrue) / (float)((int32_t)secondMeasure - firstMeasure);

				if (_k < 0.0f) _k = 1.0f;
			}

			_d = (int16_t)(((firstTrue - _k * firstMeasure) + (secondTrue - _k * secondMeasure)) / 2.0f);

			Serial.println(_k);
			Serial.println(_d);
		}

		void VL53L0::storeCalibData()
		{
			uint32_t startAddr = JAFDSettings::SpiNVSRAM::distSensStartAddr + JAFDSettings::DistanceSensors::bytesPerCalibData * _id;

			uint16_t storeK = _k * 100.0f;
			uint16_t storeD = ((uint16_t)(abs(_d)) & 0x7fff) | ((_d < 0) ? 1 << 15 : 0);

			SpiNVSRAM::writeByte(startAddr, storeK & 0xff);
			SpiNVSRAM::writeByte(startAddr + 1, storeK >> 8);
			SpiNVSRAM::writeByte(startAddr + 2, storeD & 0xff);
			SpiNVSRAM::writeByte(startAddr + 3, storeD >> 8);
		}

		void VL53L0::restoreCalibData()
		{
			uint32_t startAddr = JAFDSettings::SpiNVSRAM::distSensStartAddr + JAFDSettings::DistanceSensors::bytesPerCalibData * _id;

			uint16_t storeK = SpiNVSRAM::readByte(startAddr) | (SpiNVSRAM::readByte(startAddr + 1) << 8);
			_k = storeK / 100.0f;

			uint16_t storeD = SpiNVSRAM::readByte(startAddr + 2) | ((uint16_t)SpiNVSRAM::readByte(startAddr + 3) << 8);
			_d = (storeD & 0x7fff) * ((storeD >> 15) ? -1 : 1);

			if (_k > 1.5f || _k < 0.5f || _d > 100) {
				Serial.println("No useful distance sensor calibration data could be restored!");
				resetCalibData();
				storeCalibData();
			}
		}

		void VL53L0::resetCalibData()
		{
			_k = 1.0f;
			_d = 0;
		}

		VL53L0::Status VL53L0::getStatus() const
		{
			return _status;
		}

		void VL53L0::setKD(float k, int d)
		{
			_k = k;
			_d = d;
		}

		// VL53L0 class - end

		VL53L0 frontLeft(JAFDSettings::DistanceSensors::FrontLeft::multiplexCh, 0);
		VL53L0 frontRight(JAFDSettings::DistanceSensors::FrontRight::multiplexCh, 1);
		TFMini frontLong(JAFDSettings::DistanceSensors::FrontLong::serialType, 2);
		VL6180 leftFront(JAFDSettings::DistanceSensors::LeftFront::multiplexCh, 4);
		VL6180 leftBack(JAFDSettings::DistanceSensors::LeftBack::multiplexCh, 5);
		VL6180 rightFront(JAFDSettings::DistanceSensors::RightFront::multiplexCh, 6);
		VL6180 rightBack(JAFDSettings::DistanceSensors::RightBack::multiplexCh, 7);

		ReturnCode reset()
		{
			return setup();
		}

		ReturnCode setup()
		{
			ReturnCode code = ReturnCode::ok;

			if (leftFront.setup() != ReturnCode::ok)
			{
				Serial.println("lf");
				code = ReturnCode::fatalError;
			}

			if (leftBack.setup() != ReturnCode::ok)
			{
				Serial.println("lb");
				code = ReturnCode::fatalError;
			}

			if (rightFront.setup() != ReturnCode::ok)
			{
				Serial.println("rf");
				code = ReturnCode::fatalError;
			}

			if (rightBack.setup() != ReturnCode::ok)
			{
				Serial.println("rb");
				code = ReturnCode::fatalError;
			}

			if (frontLeft.setup() != ReturnCode::ok)
			{
				Serial.println("fl");
				code = ReturnCode::fatalError;
			}

			if (frontRight.setup() != ReturnCode::ok)
			{
				Serial.println("fr");
				code = ReturnCode::fatalError;
			}

			if (frontLong.setup() != ReturnCode::ok)
			{
				Serial.println("f");
				code = ReturnCode::fatalError;
			}

			// Read calibration data for distance sensors
			DistanceSensors::leftFront.restoreCalibData();
			DistanceSensors::leftBack.restoreCalibData();
			DistanceSensors::rightBack.restoreCalibData();
			DistanceSensors::rightFront.restoreCalibData();
			DistanceSensors::frontRight.restoreCalibData();
			DistanceSensors::frontLeft.restoreCalibData();
			DistanceSensors::frontLong.restoreCalibData();

			return code;
		}

		void updateDistSensors()
		{
			struct TempData
			{
				uint32_t tempAverageDist = 0;
				uint8_t numCorrectSamples = 0;
				uint8_t numOverflowSamples = 0;
				uint8_t numUnderflowSamples = 0;
			};

			TempData flData, frData, lfData, lbData, rfData, rbData;
			uint16_t tempDist = 0;

			auto tempFusedData = SensorFusion::getFusedData();

			// Front long
			tempDist = DistanceSensors::frontLong.getDistance();

			if (DistanceSensors::frontLong.getStatus() == decltype(DistanceSensors::frontLong)::Status::noError)
			{
				if (tempFusedData.distSensorState.frontLong == DistSensorStatus::ok)
				{
					tempFusedData.distances.frontLong = tempDist * JAFDSettings::SensorFusion::longDistSensIIRFactor + tempFusedData.distances.frontLong * (1.0f - JAFDSettings::SensorFusion::longDistSensIIRFactor);
				}
				else
				{
					tempFusedData.distances.frontLong = tempDist;
				}

				tempFusedData.distSensorState.frontLong = DistSensorStatus::ok;
			}
			else if (DistanceSensors::frontLong.getStatus() == decltype(DistanceSensors::frontLong)::Status::overflow)
			{
				tempFusedData.distSensorState.frontLong = DistSensorStatus::overflow;
				tempFusedData.distances.frontLong = 0;
			}
			else if (DistanceSensors::frontLong.getStatus() == decltype(DistanceSensors::frontLong)::Status::underflow)
			{
				tempFusedData.distSensorState.frontLong = DistSensorStatus::underflow;
				tempFusedData.distances.frontLong = 0;
			}
			else
			{
				tempFusedData.distSensorState.frontLong = DistSensorStatus::error;
				tempFusedData.distances.frontLong = 0;
			}

			tempFusedData.distSensorState.frontLong = DistSensorStatus::error;

			// Front Left
			tempDist = DistanceSensors::frontLeft.getDistance();

			if (DistanceSensors::frontLeft.getStatus() == decltype(DistanceSensors::frontLeft)::Status::noError)
			{
				flData.numCorrectSamples++;
				flData.tempAverageDist += tempDist;
			}
			else if (DistanceSensors::frontLeft.getStatus() == decltype(DistanceSensors::frontLeft)::Status::overflow)
			{
				flData.numOverflowSamples++;
			}
			else if (DistanceSensors::frontLeft.getStatus() == decltype(DistanceSensors::frontLeft)::Status::underflow)
			{
				flData.numUnderflowSamples++;
			}

			// Front Right
			tempDist = DistanceSensors::frontRight.getDistance();

			if (DistanceSensors::frontRight.getStatus() == decltype(DistanceSensors::frontRight)::Status::noError)
			{
				frData.numCorrectSamples++;
				frData.tempAverageDist += tempDist;
			}
			else if (DistanceSensors::frontRight.getStatus() == decltype(DistanceSensors::frontRight)::Status::overflow)
			{
				frData.numOverflowSamples++;
			}
			else if (DistanceSensors::frontRight.getStatus() == decltype(DistanceSensors::frontRight)::Status::underflow)
			{
				frData.numUnderflowSamples++;
			}

			// Left Back
			tempDist = DistanceSensors::leftBack.getDistance();

			if (DistanceSensors::leftBack.getStatus() == decltype(DistanceSensors::leftBack)::Status::noError)
			{
				lbData.numCorrectSamples++;
				lbData.tempAverageDist += tempDist;
			}
			else if (DistanceSensors::leftBack.getStatus() == decltype(DistanceSensors::leftBack)::Status::overflow)
			{
				lbData.numOverflowSamples++;
			}
			else if (DistanceSensors::leftBack.getStatus() == decltype(DistanceSensors::leftBack)::Status::underflow)
			{
				lbData.numUnderflowSamples++;
			}

			// Left Front
			tempDist = DistanceSensors::leftFront.getDistance();

			if (DistanceSensors::leftFront.getStatus() == decltype(DistanceSensors::leftFront)::Status::noError)
			{
				lfData.numCorrectSamples++;
				lfData.tempAverageDist += tempDist;
			}
			else if (DistanceSensors::leftFront.getStatus() == decltype(DistanceSensors::leftFront)::Status::overflow)
			{
				lfData.numOverflowSamples++;
			}
			else if (DistanceSensors::leftFront.getStatus() == decltype(DistanceSensors::leftFront)::Status::underflow)
			{
				lfData.numUnderflowSamples++;
			}

			// Right Back
			tempDist = DistanceSensors::rightBack.getDistance();

			if (DistanceSensors::rightBack.getStatus() == decltype(DistanceSensors::rightBack)::Status::noError)
			{
				rbData.numCorrectSamples++;
				rbData.tempAverageDist += tempDist;
			}
			else if (DistanceSensors::rightBack.getStatus() == decltype(DistanceSensors::rightBack)::Status::overflow)
			{
				rbData.numOverflowSamples++;
			}
			else if (DistanceSensors::rightBack.getStatus() == decltype(DistanceSensors::rightBack)::Status::underflow)
			{
				rbData.numUnderflowSamples++;
			}

			// Right Front
			tempDist = DistanceSensors::rightFront.getDistance();

			if (DistanceSensors::rightFront.getStatus() == decltype(DistanceSensors::rightFront)::Status::noError)
			{
				rfData.numCorrectSamples++;
				rfData.tempAverageDist += tempDist;
			}
			else if (DistanceSensors::rightFront.getStatus() == decltype(DistanceSensors::rightFront)::Status::overflow)
			{
				rfData.numOverflowSamples++;
			}
			else if (DistanceSensors::rightFront.getStatus() == decltype(DistanceSensors::rightFront)::Status::underflow)
			{
				rfData.numUnderflowSamples++;
			}

			if (flData.numCorrectSamples > 0)
			{
				if (tempFusedData.distSensorState.frontLeft == DistSensorStatus::ok)
				{
					tempFusedData.distances.frontLeft = static_cast<uint16_t>((static_cast<float>(flData.tempAverageDist) / flData.numCorrectSamples) * JAFDSettings::SensorFusion::shortDistSensIIRFactor + tempFusedData.distances.frontLeft * (1.0f - JAFDSettings::SensorFusion::shortDistSensIIRFactor));
				}
				else
				{
					tempFusedData.distances.frontLeft = static_cast<uint16_t>(static_cast<float>(flData.tempAverageDist) / flData.numCorrectSamples);
				}

				tempFusedData.distSensorState.frontLeft = DistSensorStatus::ok;
			}
			else
			{
				if (flData.numOverflowSamples > 0)
				{
					tempFusedData.distSensorState.frontLeft = DistSensorStatus::overflow;
				}
				else if (flData.numUnderflowSamples > 0)
				{
					tempFusedData.distSensorState.frontLeft = DistSensorStatus::underflow;
				}
				else
				{
					tempFusedData.distSensorState.frontLeft = DistSensorStatus::error;
				}

				tempFusedData.distances.frontLeft = 0;
			}

			if (frData.numCorrectSamples > 0)
			{
				if (tempFusedData.distSensorState.frontRight == DistSensorStatus::ok)
				{
					tempFusedData.distances.frontRight = static_cast<uint16_t>((static_cast<float>(frData.tempAverageDist) / frData.numCorrectSamples) * JAFDSettings::SensorFusion::shortDistSensIIRFactor + tempFusedData.distances.frontRight * (1.0f - JAFDSettings::SensorFusion::shortDistSensIIRFactor));
				}
				else
				{
					tempFusedData.distances.frontRight = static_cast<uint16_t>(static_cast<float>(frData.tempAverageDist) / frData.numCorrectSamples);
				}

				tempFusedData.distSensorState.frontRight = DistSensorStatus::ok;
			}
			else
			{
				if (frData.numOverflowSamples > 0)
				{
					tempFusedData.distSensorState.frontRight = DistSensorStatus::overflow;
				}
				else if (frData.numUnderflowSamples > 0)
				{
					tempFusedData.distSensorState.frontRight = DistSensorStatus::underflow;
				}
				else
				{
					tempFusedData.distSensorState.frontRight = DistSensorStatus::error;
				}

				tempFusedData.distances.frontRight = 0;
			}

			if (lbData.numCorrectSamples > 0)
			{
				if (tempFusedData.distSensorState.leftBack == DistSensorStatus::ok)
				{
					tempFusedData.distances.leftBack = static_cast<uint16_t>((static_cast<float>(lbData.tempAverageDist) / lbData.numCorrectSamples) * JAFDSettings::SensorFusion::shortDistSensIIRFactor + tempFusedData.distances.leftBack * (1.0f - JAFDSettings::SensorFusion::shortDistSensIIRFactor));
				}
				else
				{
					tempFusedData.distances.leftBack = static_cast<uint16_t>(static_cast<float>(lbData.tempAverageDist) / lbData.numCorrectSamples);
				}

				tempFusedData.distSensorState.leftBack = DistSensorStatus::ok;
			}
			else
			{
				if (lbData.numOverflowSamples > 0)
				{
					tempFusedData.distSensorState.leftBack = DistSensorStatus::overflow;
				}
				else if (lbData.numUnderflowSamples > 0)
				{
					tempFusedData.distSensorState.leftBack = DistSensorStatus::underflow;
				}
				else
				{
					tempFusedData.distSensorState.leftBack = DistSensorStatus::error;
				}

				tempFusedData.distances.leftBack = 0;
			}

			if (lfData.numCorrectSamples > 0)
			{
				if (tempFusedData.distSensorState.leftFront == DistSensorStatus::ok)
				{
					tempFusedData.distances.leftFront = static_cast<uint16_t>((static_cast<float>(lfData.tempAverageDist) / lfData.numCorrectSamples) * JAFDSettings::SensorFusion::shortDistSensIIRFactor + tempFusedData.distances.leftFront * (1.0f - JAFDSettings::SensorFusion::shortDistSensIIRFactor));
				}
				else
				{
					tempFusedData.distances.leftFront = static_cast<uint16_t>(static_cast<float>(lfData.tempAverageDist) / lfData.numCorrectSamples);
				}

				tempFusedData.distSensorState.leftFront = DistSensorStatus::ok;
			}
			else
			{
				if (lfData.numOverflowSamples > 0)
				{
					tempFusedData.distSensorState.leftFront = DistSensorStatus::overflow;
				}
				else if (lfData.numUnderflowSamples > 0)
				{
					tempFusedData.distSensorState.leftFront = DistSensorStatus::underflow;
				}
				else
				{
					tempFusedData.distSensorState.leftFront = DistSensorStatus::error;
				}

				tempFusedData.distances.leftFront = 0;
			}

			if (rbData.numCorrectSamples > 0)
			{
				if (tempFusedData.distSensorState.rightBack == DistSensorStatus::ok)
				{
					tempFusedData.distances.rightBack = static_cast<uint16_t>((static_cast<float>(rbData.tempAverageDist) / rbData.numCorrectSamples) * JAFDSettings::SensorFusion::shortDistSensIIRFactor + tempFusedData.distances.rightBack * (1.0f - JAFDSettings::SensorFusion::shortDistSensIIRFactor));
				}
				else
				{
					tempFusedData.distances.rightBack = static_cast<uint16_t>(static_cast<float>(rbData.tempAverageDist) / rbData.numCorrectSamples);
				}

				tempFusedData.distSensorState.rightBack = DistSensorStatus::ok;
			}
			else
			{
				if (rbData.numOverflowSamples > 0)
				{
					tempFusedData.distSensorState.rightBack = DistSensorStatus::overflow;
				}
				else if (rbData.numUnderflowSamples > 0)
				{
					tempFusedData.distSensorState.rightBack = DistSensorStatus::underflow;
				}
				else
				{
					tempFusedData.distSensorState.rightBack = DistSensorStatus::error;
				}

				tempFusedData.distances.rightBack = 0;
			}

			if (rfData.numCorrectSamples > 0)
			{
				if (tempFusedData.distSensorState.rightFront == DistSensorStatus::ok)
				{
					tempFusedData.distances.rightFront = static_cast<uint16_t>((static_cast<float>(rfData.tempAverageDist) / rfData.numCorrectSamples) * JAFDSettings::SensorFusion::shortDistSensIIRFactor + tempFusedData.distances.rightFront * (1.0f - JAFDSettings::SensorFusion::shortDistSensIIRFactor));
				}
				else
				{
					tempFusedData.distances.rightFront = static_cast<uint16_t>(static_cast<float>(rfData.tempAverageDist) / rfData.numCorrectSamples);
				}

				tempFusedData.distSensorState.rightFront = DistSensorStatus::ok;
			}
			else
			{
				if (rfData.numOverflowSamples > 0)
				{
					tempFusedData.distSensorState.rightFront = DistSensorStatus::overflow;
				}
				else if (rfData.numUnderflowSamples > 0)
				{
					tempFusedData.distSensorState.rightFront = DistSensorStatus::underflow;
				}
				else
				{
					tempFusedData.distSensorState.rightFront = DistSensorStatus::error;
				}

				tempFusedData.distances.rightFront = 0;
			}

			SensorFusion::setDistances(tempFusedData.distances);
			SensorFusion::setDistSensStates(tempFusedData.distSensorState);
		}

		void forceNewMeasurement()
		{
			frontLeft.clearInterrupt();
			frontRight.clearInterrupt();
			leftFront.clearInterrupt();
			leftBack.clearInterrupt();
			rightFront.clearInterrupt();
			rightBack.clearInterrupt();
		}

		float getSingleCalibData(int sensorId)
		{
			switch (sensorId)
			{
			case 0:
				DistanceSensors::rightBack.resetCalibData();
				break;
			case 1:
				DistanceSensors::rightFront.resetCalibData();
				break;
			case 2:
				DistanceSensors::leftBack.resetCalibData();
				break;
			case 3:
				DistanceSensors::leftFront.resetCalibData();
				break;
			case 4:
				DistanceSensors::frontLeft.resetCalibData();
				break;
			case 5:
				DistanceSensors::frontRight.resetCalibData();
				break;
			case 6:
				DistanceSensors::frontLong.resetCalibData();
				break;
			default:
				break;
			}

			auto startCalibTime = millis();
			float avgDist = 0.0f;
			uint16_t avgCount = 0;

			while (millis() - startCalibTime < 2000)
			{
				SensorFusion::updateSensors();
				SensorFusion::untimedFusion();
				auto fusedData = SensorFusion::getFusedData();

				DistSensorStatus state = DistSensorStatus::error;

				switch (sensorId)
				{
				case 0:
					state = fusedData.distSensorState.rightBack;
					break;
				case 1:
					state = fusedData.distSensorState.rightFront;
					break;
				case 2:
					state = fusedData.distSensorState.leftBack;
					break;
				case 3:
					state = fusedData.distSensorState.leftFront;
					break;
				case 4:
					state = fusedData.distSensorState.frontLeft;
					break;
				case 5:
					state = fusedData.distSensorState.frontRight;
					break;
				case 6:
					state = fusedData.distSensorState.frontLong;
					break;
				default:
					break;
				}

				if (state == DistSensorStatus::ok)
				{
					switch (sensorId)
					{
					case 0:
						avgDist += fusedData.distances.rightBack;
						break;
					case 1:
						avgDist += fusedData.distances.rightFront;
						break;
					case 2:
						avgDist += fusedData.distances.leftBack;
						break;
					case 3:
						avgDist += fusedData.distances.leftFront;
						break;
					case 4:
						avgDist += fusedData.distances.frontLeft;
						break;
					case 5:
						avgDist += fusedData.distances.frontRight;
						break;
					case 6:
						avgDist += fusedData.distances.frontLong;
						break;
					default:
						break;
					}
					
					avgCount++;
				}
			}

			return avgDist / (float)avgCount;
		}

		void averagedCalibration()
		{
			/*
			* calibration data:
				rb
					k: 0.99
					d: 7
				rf
					k: 1.01
					d: 7
				lb
					k: 1.04
					d: -11
				lf
					k: 1.01
					d: 0
				fl
					k: 0.97
					d: -13
				fr
					k: 0.97
					d: -25
			*/

			for (int i = 0; i <= 6; i++)
			{
				switch (i)
				{
				case 0:
					Serial.println("rb");
					break;
				case 1:
					Serial.println("rf");
					break;
				case 2:
					Serial.println("lb");
					break;
				case 3:
					Serial.println("lf");
					break;
				case 4:
					Serial.println("fl");
					break;
				case 5:
					Serial.println("fr");
					break;
				case 6:
					Serial.println("flong");
					break;
				default:
					break;
				}

				while (Serial.available()) Serial.read();
				while (!Serial.available());
				delay(10);
				while (Serial.available()) Serial.read();

				float measFirst = getSingleCalibData(i);
				Serial.println(measFirst);
				while (!Serial.available());
				delay(10);
				uint16_t trueFirst = (uint16_t)Serial.parseInt();

				Serial.println("begin second measurement");

				while (Serial.available()) Serial.read();
				while (!Serial.available());
				delay(10);
				while (Serial.available()) Serial.read();

				float measSecond = getSingleCalibData(i);
				Serial.println(measSecond);
				while (!Serial.available());
				delay(10);
				uint16_t trueSecond = (uint16_t)Serial.parseInt();

				switch (i)
				{
				case 0:
					DistanceSensors::rightBack.calcCalibData(trueFirst, measFirst, trueSecond, measSecond);
					DistanceSensors::rightBack.storeCalibData();
					break;
				case 1:
					DistanceSensors::rightFront.calcCalibData(trueFirst, measFirst, trueSecond, measSecond);
					DistanceSensors::rightFront.storeCalibData();
					break;
				case 2:
					DistanceSensors::leftBack.calcCalibData(trueFirst, measFirst, trueSecond, measSecond);
					DistanceSensors::leftBack.storeCalibData();
					break;
				case 3:
					DistanceSensors::leftFront.calcCalibData(trueFirst, measFirst, trueSecond, measSecond);
					DistanceSensors::leftFront.storeCalibData();
					break;
				case 4:
					DistanceSensors::frontLeft.calcCalibData(trueFirst, measFirst, trueSecond, measSecond);
					DistanceSensors::frontLeft.storeCalibData();
					break;
				case 5:
					DistanceSensors::frontRight.calcCalibData(trueFirst, measFirst, trueSecond, measSecond);
					DistanceSensors::frontRight.storeCalibData();
					break;
				case 6:
					DistanceSensors::frontLong.calcCalibData(trueFirst, measFirst, trueSecond, measSecond);
					DistanceSensors::frontLong.storeCalibData();
					break;
				default:
					break;
				}
			}
		}
	}
}