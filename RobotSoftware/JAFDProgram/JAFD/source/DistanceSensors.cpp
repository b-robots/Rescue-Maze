#include "../header/DistanceSensors.h"

namespace JAFD
{
	A::A(uint8_t address)
	{}

	namespace DistanceSensors
	{
		A front = A(JAFDSettings::DistanceSensors::Front::i2cAddress);
		B left = B();
	}
}