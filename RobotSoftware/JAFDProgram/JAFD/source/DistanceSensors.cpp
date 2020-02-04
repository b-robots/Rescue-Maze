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
		VL6180::VL6180(uint8_t multiplexCh) : _multiplexCh(multiplexCh) {}

		ReturnCode VL6180::setup() const
		{
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
			if (_i2cMultiplexer.getChannel() != _multiplexCh)
			{
				_i2cMultiplexer.selectChannel(_multiplexCh);
			}

			Wire.beginTransmission(_i2cAddr);
			Wire.write(address >> 8);
			Wire.write(address);
			Wire.write(data);
			Wire.endTransmission();
		}

		// Write 2 bytes
		void VL6180::write16(uint16_t address, uint16_t data) const
		{
			if (_i2cMultiplexer.getChannel() != _multiplexCh)
			{
				_i2cMultiplexer.selectChannel(_multiplexCh);
			}

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
			if (_i2cMultiplexer.getChannel() != _multiplexCh)
			{
				_i2cMultiplexer.selectChannel(_multiplexCh);
			}

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
			if (_i2cMultiplexer.getChannel() != _multiplexCh)
			{
				_i2cMultiplexer.selectChannel(_multiplexCh);
			}

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

		void VL6180::updateValues()
		{
			// Wait for device to be ready for range measurement
			while (!(read8(_regRangeStatus) & 0x01));

			// Start a range measurement
			write8(_regRangeStart, 0x01);

			// Poll until bit 2 is set
			while (!(read8(_regIntStatus) & 0x04));

			// Read range in mm
			_distance = read8(_regRangeResult);

			// Clear interrupt
			write8(_regIntClear, 0x07);

			// Read status
			_status = static_cast<VL6180::Status>(read8(_regRangeStatus) >> 4);
		}

		uint8_t VL6180::getDistance() const
		{
			return _distance;
		}

		VL6180::Status VL6180::getStatus() const
		{
			return _status;
		}

		// VL6180 class - end

		// MyTFMini class - begin

		/*MyTFMini::MyTFMini(SerialType serialType) : _serialType(serialType) {}

		ReturnCode MyTFMini::setup()
		{
			switch (_serialType)
			{
			case JAFD::SerialType::software:
				return ReturnCode::error;
				break;

			case JAFD::SerialType::zero:
				Serial.begin(TFMINI_BAUDRATE);
				_sensor.begin(&Serial);
				break;

			case JAFD::SerialType::one:
				Serial1.begin(TFMINI_BAUDRATE);
				_sensor.begin(&Serial1);
				break;

			case JAFD::SerialType::two:
				Serial2.begin(TFMINI_BAUDRATE);
				_sensor.begin(&Serial2);
				break;

			case JAFD::SerialType::three:
				Serial3.begin(TFMINI_BAUDRATE);
				_sensor.begin(&Serial3);
				break;

			default:
				return ReturnCode::error;
			}
			
			return ReturnCode::ok;
		}*/

		/*void MyTFMini::updateValues()
		{
			_distance = _sensor.getDistance();
			
			if (_sensor.getState() != MEASUREMENT_OK)
			{
				_status = Status::unknownError;
				Serial.println("Fehler");
			}
			else
			{
				_status = Status::noError;
			}
		}

		uint16_t MyTFMini::getDistance() const
		{
			return _distance;
		}

		Status MyTFMini::getStatus() const
		{
			return _status;
		}*/

		// TFMini class - end

		VL6180 frontLeft(JAFDSettings::DistanceSensors::FrontLeft::multiplexCh);
		VL6180 frontRight(JAFDSettings::DistanceSensors::FrontRight::multiplexCh);
		//MyTFMini frontLong(JAFDSettings::DistanceSensors::FrontLong::serialType);
		//MyTFMini backLong(JAFDSettings::DistanceSensors::BackLong::serialType);
		VL6180 leftFront(0);
		VL6180 leftBack(0);
		VL6180 rightFront(0);
		VL6180 rightBack(0);

		ReturnCode setup()
		{
			ReturnCode code = ReturnCode::ok;
			
			if (frontLeft.setup() != ReturnCode::ok)
			{
				Serial.println("left");
				code = ReturnCode::fatalError;
			}

			if (frontRight.setup() != ReturnCode::ok)
			{
				Serial.println("right");
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

			//if (leftFront.setup() != ReturnCode::ok)
			//{
			//	code = ReturnCode::fatalError;
			//}

			//if (leftBack.setup() != ReturnCode::ok)
			//{
			//	code = ReturnCode::fatalError;
			//}

			//if (rightFront.setup() != ReturnCode::ok)
			//{
			//	code = ReturnCode::fatalError;
			//}

			//if (rightBack.setup() != ReturnCode::ok)
			//{
			//	code = ReturnCode::fatalError;
			//}

			return code;
		}
	}
}