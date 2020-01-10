#include "../header/DistanceSensors.h"

namespace JAFD
{
	VL6180::VL6180(uint8_t address)
	{
	
	}

	void VL6180::updateValues()
	{
		_distance = _sensor.readRange();
		_surroundLight = _sensor.readLux(VL6180X_ALS_GAIN_1);
		_status = _sensor.readRangeStatus();
	}

	float VL6180::getDistance()
	{
		return _distance;
	}

	uint8_t VL6180::getStatus()
	{
		return _status;
	}

	namespace DistanceSensors
	{
		VL6180 front = VL6180(JAFDSettings::DistanceSensors::Front::i2cAddress);
		Lidar left = Lidar();
	}
}