/*
This file is responsible for all distance sensors
*/

#include "../header/DistanceSensors.h"

namespace JAFD
{
	namespace DistanceSensors
	{
		// VL6180 class - begin

		ReturnCode VL6180::setup()
		{
			if (_sensor.begin()) return ReturnCode::ok;
			else return ReturnCode::fatalError;
		}

		void VL6180::updateValues()
		{
			_distance = _sensor.readRange();

			auto status = _sensor.readRangeStatus();

			if ((status >= VL6180X_ERROR_SYSERR_1) && (status <= VL6180X_ERROR_SYSERR_5)) {
				_status = Status::systemError;
			}
			else if (status == VL6180X_ERROR_ECEFAIL) {
				_status = Status::eceFailure;
			}
			else if (status == VL6180X_ERROR_NOCONVERGE) {
				_status = Status::noConvergence;
			}
			else if (status == VL6180X_ERROR_RANGEIGNORE) {
				_status = Status::ignoringRange;
			}
			else if (status == VL6180X_ERROR_SNR) {
				_status = Status::noiseError;
			}
			else if (status == VL6180X_ERROR_RAWUFLOW || status == VL6180X_ERROR_RANGEUFLOW) {
				_status = Status::underflow;
			}
			else if (status == VL6180X_ERROR_RAWOFLOW || status == VL6180X_ERROR_RANGEOFLOW) {
				_status = Status::overflow;
			}
			else
			{
				_status = Status::noError;
			}
		}

		uint16_t VL6180::getDistance() const
		{
			return _distance;
		}

		Status VL6180::getStatus() const
		{
			return _status;
		}

		// VL6180 class - end

		// MyTFMini class - begin

		MyTFMini::MyTFMini(SerialType serialType) : _serialType(serialType) {}

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
		}

		void MyTFMini::updateValues()
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
		}

		// TFMini class - end

		VL6180 frontLeft;
		VL6180 frontRight;
		MyTFMini frontLong(JAFDSettings::DistanceSensors::FrontLong::serialType);
		MyTFMini backLong(JAFDSettings::DistanceSensors::BackLong::serialType);
		VL6180 leftFront;
		VL6180 leftBack;
		VL6180 rightFront;
		VL6180 rightBack;

		ReturnCode setup()
		{
			ReturnCode code = ReturnCode::ok;
			
			if (frontLeft.setup() != ReturnCode::ok)
			{
				code = ReturnCode::fatalError;
			}

			//if (frontRight.setup() != ReturnCode::ok)
			//{
			//	code = ReturnCode::fatalError;
			//}

			if (frontLong.setup() != ReturnCode::ok)
			{
				code = ReturnCode::fatalError;
			}

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