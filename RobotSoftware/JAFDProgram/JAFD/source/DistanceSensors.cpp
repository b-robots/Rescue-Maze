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

		float VL6180::getDistance() const
		{
			return _distance;
		}

		Status VL6180::getStatus() const
		{
			return _status;
		}

		// VL6180 class - end

		// Lidar class - begin

		ReturnCode Lidar::setup()
		{
			if (_sensor.begin()) return ReturnCode::ok;
			else return ReturnCode::fatalError;
		}

		void Lidar::updateValues()
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

		float Lidar::getDistance() const
		{
			return _distance;
		}

		Status Lidar::getStatus() const
		{
			return _status;
		}

		// Lidar class - end

		VL6180 front = VL6180();
		Lidar left = Lidar();

		ReturnCode setup()
		{
			ReturnCode code = ReturnCode::ok;
			
			if (front.setup() != ReturnCode::ok)
			{
				code = ReturnCode::fatalError;
			}

			if (left.setup() != ReturnCode::ok)
			{
				code = ReturnCode::fatalError;
			}

			return code;
		}
	}
}