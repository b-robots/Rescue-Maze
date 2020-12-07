#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include "../../JAFDSettings.h"
#include "../header/SmallThings.h"
#include "../header/DistanceSensors.h"
#include "../header/HeatSensor.h"

namespace JAFD
{
	namespace PowerLEDs
	{
		namespace
		{
			constexpr auto lPWM = PinMapping::MappedPins[JAFDSettings::PowerLEDs::Left::pwmPin];	// PWM pin left LED
			constexpr auto rPWM = PinMapping::MappedPins[JAFDSettings::PowerLEDs::Right::pwmPin];	// PWM pin right LED
			
			constexpr uint8_t lPWMCh = PinMapping::getPWMChannel(lPWM);		// Left LED PWM channel
			constexpr uint8_t rPWMCh = PinMapping::getPWMChannel(rPWM);		// Left LED PWM channel
		}

		ReturnCode setup()
		{
			// Setup PWM - Controller (~200Hz)
			PWM->PWM_ENA = 1 << lPWMCh | 1 << rPWMCh;

			PWM->PWM_CH_NUM[lPWMCh].PWM_CMR = PWM_CMR_CPRE_MCK_DIV_64;
			PWM->PWM_CH_NUM[lPWMCh].PWM_CPRD = 6562;
			PWM->PWM_CH_NUM[lPWMCh].PWM_CDTY = 0;

			if (PinMapping::getPWMStartState(lPWM) == PinMapping::PWMStartState::high)
			{
				PWM->PWM_CH_NUM[lPWMCh].PWM_CMR |= PWM_CMR_CPOL;
			}

			PWM->PWM_CH_NUM[rPWMCh].PWM_CMR = PWM_CMR_CPRE_MCK_DIV_64;
			PWM->PWM_CH_NUM[rPWMCh].PWM_CPRD = 6562;
			PWM->PWM_CH_NUM[rPWMCh].PWM_CDTY = 0;

			if (PinMapping::getPWMStartState(rPWM) == PinMapping::PWMStartState::high)
			{
				PWM->PWM_CH_NUM[rPWMCh].PWM_CMR |= PWM_CMR_CPOL;
			}

			lPWM.port->PIO_PDR = lPWM.pin;
			rPWM.port->PIO_PDR = rPWM.pin;

			if (PinMapping::toABPeripheral(lPWM))
			{
				lPWM.port->PIO_ABSR |= lPWM.pin;
			}
			else
			{
				lPWM.port->PIO_ABSR &= ~lPWM.pin;
			}

			if (PinMapping::toABPeripheral(rPWM))
			{
				rPWM.port->PIO_ABSR |= rPWM.pin;
			}
			else
			{
				rPWM.port->PIO_ABSR &= ~rPWM.pin;
			}

			// Make pins Open Drain
			lPWM.port->PIO_PUDR = lPWM.pin;
			rPWM.port->PIO_PUDR = rPWM.pin;
			lPWM.port->PIO_MDER = lPWM.pin;
			rPWM.port->PIO_MDER = rPWM.pin;

			setBrightness(0.0f);

			return ReturnCode::ok;
		}

		void setBrightness(float perc)
		{
			PWM->PWM_CH_NUM[lPWMCh].PWM_CDTYUPD = static_cast<uint16_t>(PWM->PWM_CH_NUM[lPWMCh].PWM_CPRD * perc);
			PWM->PWM_CH_NUM[rPWMCh].PWM_CDTYUPD = static_cast<uint16_t>(PWM->PWM_CH_NUM[rPWMCh].PWM_CPRD * perc);
			PWM->PWM_SCUC = PWM_SCUC_UPDULOCK;
		}
	}

	namespace I2CBus
	{
		namespace
		{
			constexpr auto resetBusPin = PinMapping::MappedPins[JAFDSettings::I2CBus::powerResetPin];
		}

		ReturnCode setup()
		{
			resetBusPin.port->PIO_PER = resetBusPin.pin;
			resetBusPin.port->PIO_OER = resetBusPin.pin;

			resetBusPin.port->PIO_SODR = resetBusPin.pin;

			return ReturnCode::ok;
		}

		ReturnCode resetBus()
		{
			ReturnCode result = ReturnCode::ok;

			resetBusPin.port->PIO_CODR = resetBusPin.pin;
			delay(5);
			resetBusPin.port->PIO_SODR = resetBusPin.pin;

			if (DistanceSensors::reset() != ReturnCode::ok) result = ReturnCode::error;

			if (HeatSensor::reset() != ReturnCode::ok) result = ReturnCode::error;

			return result;
		}
	}
}