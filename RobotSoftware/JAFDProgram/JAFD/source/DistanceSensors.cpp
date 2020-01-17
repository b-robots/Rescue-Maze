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

		// Lidar class - begin

		ReturnCode TFMini::setup()
		{
			_sensor.begin(Serial1);
			return ReturnCode::ok;
		}

		void TFMini::updateValues()
		{
			_distance = _sensor.getDistance();

		}

		uint16_t TFMini::getDistance() const
		{
			return _distance;
		}

		// Lidar class - end

		VL6180 frontLeft;
		VL6180 frontRight;
		TFMini frontLong;
		TFMini backLong;
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

			if (frontRight.setup() != ReturnCode::ok)
			{
				code = ReturnCode::fatalError;
			}

			if (frontLong.setup() != ReturnCode::ok)
			{
				code = ReturnCode::fatalError;
			}

			if (backLong.setup() != ReturnCode::ok)
			{
				code = ReturnCode::fatalError;
			}

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

			return code;
		}
	}
}